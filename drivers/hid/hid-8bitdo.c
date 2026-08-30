// SPDX-License-Identifier: GPL-2.0-only AND Zlib
/*
 * HID driver for 8BitDo controllers using the SDL report protocol.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "hid-ids.h"

#define BDO_INPUT_VERSION			0x8bd0

#define BDO_FEATURE_REPORT_ID			0x30
#define BDO_ENABLE_SDL_REPORT_ID		0x06
#define BDO_SDL_USB_REPORT_ID			0x04
#define BDO_SDL_UNSUPPORTED_REPORT_ID		0x03
#define BDO_SDL_BT_REPORT_ID			0x01
#define BDO_OUTPUT_REPORT_ID			0x05

#define BDO_SENSOR_TIMESTAMP_ENABLED		0xaa
#define BDO_FEATURE_RETRIES			5
#define BDO_FEATURE_RETRY_MS			10
#define BDO_FEATURE_REPORT_SIZE			64
#define BDO_ULTIMATE2_REPORT_SIZE		34
#define BDO_ULTIMATE2_WAIT_MS			240

#define BDO_STATE_MIN_SIZE			10
#define BDO_SENSOR_MIN_SIZE			27
#define BDO_TIMESTAMP_MIN_SIZE			31

#define BDO_ACCEL_RANGE				32768
#define BDO_ACCEL_RES_PER_G			4096
#define BDO_GYRO_SCALE				1000
#define BDO_GYRO_RANGE				(BDO_ACCEL_RANGE * BDO_GYRO_SCALE)
#define BDO_GYRO_RES_PER_DPS			16384

enum bdo_model {
	BDO_SF30_SN30,
	BDO_PRO,
	BDO_ULTIMATE2,
	BDO_ULTIMATE3,
};

struct bdo_device {
	struct hid_device *hdev;
	struct input_dev *gamepad;
	struct input_dev *motion;
	struct power_supply *battery;
	struct power_supply_desc battery_desc;
	struct completion first_report;
	struct work_struct rumble_work;
	spinlock_t lock; /* protects battery and rumble state */
	enum bdo_model model;
	char *uniq;
	u8 first_report_size;
	u8 battery_capacity;
	int battery_status;
	u8 rumble_strong;
	u8 rumble_weak;
	bool first_report_seen;
	bool sensors_supported;
	bool sensor_timestamp_supported;
	bool rumble_supported;
	bool power_supported;
	bool timestamp_initialized;
	bool removed;
	u32 previous_sensor_tick;
	u32 sensor_timestamp_us;
	u32 sensor_interval_us;
};

static bool bdo_is_gamepad(const struct hid_device *hdev)
{
	unsigned int i;

	for (i = 0; i < hdev->maxcollection; i++) {
		const struct hid_collection *collection = &hdev->collection[i];

		if (collection->type == HID_COLLECTION_APPLICATION &&
		    (collection->usage == HID_GD_GAMEPAD ||
		     collection->usage == HID_GD_JOYSTICK))
			return true;
	}

	return false;
}

static s32 bdo_scale_stick(u8 value)
{
	if (value <= 0x7f)
		return DIV_ROUND_CLOSEST((s32)(value - 0x7f) * 32768, 0x7f);

	return DIV_ROUND_CLOSEST((s32)(value - 0x7f) * 32767, 0x80);
}

static void bdo_report_dpad(struct input_dev *gamepad, u8 value)
{
	static const s8 directions[8][2] = {
		{  0, -1 }, {  1, -1 }, {  1,  0 }, {  1,  1 },
		{  0,  1 }, { -1,  1 }, { -1,  0 }, { -1, -1 },
	};
	s8 x = 0;
	s8 y = 0;

	if (value < ARRAY_SIZE(directions)) {
		x = directions[value][0];
		y = directions[value][1];
	}

	input_report_abs(gamepad, ABS_HAT0X, x);
	input_report_abs(gamepad, ABS_HAT0Y, y);
}

static void bdo_report_enhanced_gamepad(struct bdo_device *bdo,
					const u8 *data, int size)
{
	struct input_dev *gamepad = bdo->gamepad;

	if (!gamepad || size < BDO_STATE_MIN_SIZE)
		return;

	bdo_report_dpad(gamepad, data[1]);
	input_report_abs(gamepad, ABS_X, bdo_scale_stick(data[2]));
	input_report_abs(gamepad, ABS_Y, bdo_scale_stick(data[3]));
	input_report_abs(gamepad, ABS_RX, bdo_scale_stick(data[4]));
	input_report_abs(gamepad, ABS_RY, bdo_scale_stick(data[5]));
	input_report_abs(gamepad, ABS_Z, data[7]);
	input_report_abs(gamepad, ABS_RZ, data[6]);

	input_report_key(gamepad, BTN_A, data[8] & BIT(1));
	input_report_key(gamepad, BTN_B, data[8] & BIT(0));
	input_report_key(gamepad, BTN_X, data[8] & BIT(4));
	input_report_key(gamepad, BTN_Y, data[8] & BIT(3));
	input_report_key(gamepad, BTN_TL, data[8] & BIT(6));
	input_report_key(gamepad, BTN_TR, data[8] & BIT(7));
	if (bdo->model != BDO_SF30_SN30) {
		input_report_key(gamepad, BTN_GRIPL, data[8] & BIT(5));
		input_report_key(gamepad, BTN_GRIPR, data[8] & BIT(2));
	}

	input_report_key(gamepad, BTN_SELECT, data[9] & BIT(2));
	input_report_key(gamepad, BTN_START, data[9] & BIT(3));
	input_report_key(gamepad, BTN_MODE, data[9] & BIT(4));
	input_report_key(gamepad, BTN_THUMBL, data[9] & BIT(5));
	input_report_key(gamepad, BTN_THUMBR, data[9] & BIT(6));

	if (bdo->model != BDO_SF30_SN30 && size > 10) {
		input_report_key(gamepad, BTN_GRIPL2, data[10] & BIT(0));
		input_report_key(gamepad, BTN_GRIPR2, data[10] & BIT(1));
	}
	if (bdo->model == BDO_ULTIMATE3)
		input_report_key(gamepad, KEY_RECORD, data[9] & BIT(7));

	input_sync(gamepad);
}

static void bdo_report_standard_gamepad(struct bdo_device *bdo,
					const u8 *data, int size)
{
	struct input_dev *gamepad = bdo->gamepad;
	u16 buttons;

	if (!gamepad || size < 10)
		return;

	bdo_report_dpad(gamepad, data[1] & 0x0f);
	input_report_abs(gamepad, ABS_X, bdo_scale_stick(data[2]));
	input_report_abs(gamepad, ABS_Y, bdo_scale_stick(data[3]));
	input_report_abs(gamepad, ABS_RX, bdo_scale_stick(data[4]));
	input_report_abs(gamepad, ABS_RY, bdo_scale_stick(data[5]));
	input_report_abs(gamepad, ABS_Z, data[7]);
	input_report_abs(gamepad, ABS_RZ, data[6]);

	buttons = get_unaligned_le16(data + 8);
	input_report_key(gamepad, BTN_A, buttons & BIT(1));
	input_report_key(gamepad, BTN_B, buttons & BIT(0));
	input_report_key(gamepad, BTN_X, buttons & BIT(4));
	input_report_key(gamepad, BTN_Y, buttons & BIT(3));
	input_report_key(gamepad, BTN_TL, buttons & BIT(6));
	input_report_key(gamepad, BTN_TR, buttons & BIT(7));
	input_report_key(gamepad, BTN_SELECT, buttons & BIT(10));
	input_report_key(gamepad, BTN_START, buttons & BIT(11));
	input_report_key(gamepad, BTN_MODE, buttons & (BIT(2) | BIT(12)));
	input_report_key(gamepad, BTN_THUMBL, buttons & BIT(13));
	input_report_key(gamepad, BTN_THUMBR, buttons & BIT(14));
	input_sync(gamepad);
}

static void bdo_report_legacy_gamepad(struct bdo_device *bdo,
				      const u8 *data, int size)
{
	struct input_dev *gamepad = bdo->gamepad;

	if (!gamepad || size != 9)
		return;

	bdo_report_dpad(gamepad, data[2]);
	input_report_abs(gamepad, ABS_X, bdo_scale_stick(data[3]));
	input_report_abs(gamepad, ABS_Y, bdo_scale_stick(data[4]));
	input_report_abs(gamepad, ABS_RX, bdo_scale_stick(data[5]));
	input_report_abs(gamepad, ABS_RY, bdo_scale_stick(data[6]));
	input_report_abs(gamepad, ABS_Z, (data[1] & BIT(0)) ? 255 : 0);
	input_report_abs(gamepad, ABS_RZ, (data[1] & BIT(1)) ? 255 : 0);

	input_report_key(gamepad, BTN_A, data[0] & BIT(1));
	input_report_key(gamepad, BTN_B, data[0] & BIT(0));
	input_report_key(gamepad, BTN_X, data[0] & BIT(4));
	input_report_key(gamepad, BTN_Y, data[0] & BIT(3));
	input_report_key(gamepad, BTN_TL, data[0] & BIT(6));
	input_report_key(gamepad, BTN_TR, data[0] & BIT(7));
	input_report_key(gamepad, BTN_SELECT, data[1] & BIT(2));
	input_report_key(gamepad, BTN_START, data[1] & BIT(3));
	input_report_key(gamepad, BTN_MODE, data[1] & BIT(4));
	input_report_key(gamepad, BTN_THUMBL, data[1] & BIT(5));
	input_report_key(gamepad, BTN_THUMBR, data[1] & BIT(6));
	input_sync(gamepad);
}

static u32 bdo_sensor_interval_us(const struct bdo_device *bdo)
{
	u32 rate;

	switch (bdo->model) {
	case BDO_SF30_SN30:
		rate = bdo->hdev->bus == BUS_BLUETOOTH ? 70 :
		       (bdo->sensor_timestamp_supported ? 200 : 100);
		break;
	case BDO_PRO:
		rate = bdo->hdev->bus == BUS_BLUETOOTH ? 85 :
		       (bdo->sensor_timestamp_supported ? 200 : 100);
		break;
	case BDO_ULTIMATE2:
		rate = bdo->hdev->bus == BUS_BLUETOOTH ? 120 : 1000;
		break;
	case BDO_ULTIMATE3:
	default:
		rate = 120;
		break;
	}

	return DIV_ROUND_CLOSEST(USEC_PER_SEC, rate);
}

static u32 bdo_sensor_timestamp(struct bdo_device *bdo,
				const u8 *data, int size)
{
	u32 delta = bdo->sensor_interval_us;

	if (bdo->sensor_timestamp_supported && size >= BDO_TIMESTAMP_MIN_SIZE) {
		u32 tick = get_unaligned_le32(data + 27);

		if (!bdo->timestamp_initialized) {
			bdo->previous_sensor_tick = tick;
			bdo->timestamp_initialized = true;
		} else {
			u32 device_delta = tick - bdo->previous_sensor_tick;

			if (device_delta && device_delta < 100000)
				delta = device_delta;
			bdo->previous_sensor_tick = tick;
		}
	}

	bdo->sensor_timestamp_us += delta;
	return bdo->sensor_timestamp_us;
}

static void bdo_report_motion(struct bdo_device *bdo, const u8 *data,
			      int size)
{
	struct input_dev *motion = bdo->motion;
	s16 accel_x, accel_y, accel_z;
	s16 gyro_x, gyro_y, gyro_z;

	if (!motion || size < BDO_SENSOR_MIN_SIZE)
		return;

	accel_x = get_unaligned_le16(data + 15);
	accel_y = get_unaligned_le16(data + 17);
	accel_z = get_unaligned_le16(data + 19);
	gyro_x = get_unaligned_le16(data + 21);
	gyro_y = get_unaligned_le16(data + 23);
	gyro_z = get_unaligned_le16(data + 25);

	input_report_abs(motion, ABS_X, -(s32)accel_y);
	input_report_abs(motion, ABS_Y, accel_z);
	input_report_abs(motion, ABS_Z, -(s32)accel_x);
	input_report_abs(motion, ABS_RX, -(s32)gyro_y * BDO_GYRO_SCALE);
	input_report_abs(motion, ABS_RY, gyro_z * BDO_GYRO_SCALE);
	input_report_abs(motion, ABS_RZ, -(s32)gyro_x * BDO_GYRO_SCALE);
	input_event(motion, EV_MSC, MSC_TIMESTAMP,
		    bdo_sensor_timestamp(bdo, data, size));
	input_sync(motion);
}

static void bdo_update_battery(struct bdo_device *bdo, u8 value)
{
	unsigned long flags;
	u8 capacity = value & 0x7f;
	int status;
	bool changed;

	if (!bdo->battery)
		return;

	capacity = min_t(u8, capacity, 100);
	if (capacity == 100)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (value & BIT(7))
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	spin_lock_irqsave(&bdo->lock, flags);
	changed = capacity != bdo->battery_capacity ||
		  status != bdo->battery_status;
	bdo->battery_capacity = capacity;
	bdo->battery_status = status;
	spin_unlock_irqrestore(&bdo->lock, flags);

	if (changed)
		power_supply_changed(bdo->battery);
}

static int bdo_raw_event(struct hid_device *hdev, struct hid_report *report,
			 u8 *data, int size)
{
	struct bdo_device *bdo = hid_get_drvdata(hdev);
	u8 report_id;

	if (!bdo || report->type != HID_INPUT_REPORT || size <= 0)
		return 0;

	if (bdo->model == BDO_ULTIMATE2 && !bdo->first_report_seen) {
		bdo->first_report_size = min_t(int, size, U8_MAX);
		bdo->first_report_seen = true;
		complete(&bdo->first_report);
	}

	if (size == 9) {
		bdo_report_legacy_gamepad(bdo, data, size);
		return 1;
	}

	report_id = data[0];
	if (report_id == BDO_SDL_UNSUPPORTED_REPORT_ID && size == 10) {
		bdo_report_standard_gamepad(bdo, data, size);
		return 1;
	}

	if (report_id != BDO_SDL_USB_REPORT_ID &&
	    report_id != BDO_SDL_BT_REPORT_ID &&
	    report_id != BDO_SDL_UNSUPPORTED_REPORT_ID)
		return 0;

	if (size < BDO_STATE_MIN_SIZE)
		return 1;

	bdo_report_enhanced_gamepad(bdo, data, size);
	if (size > 14)
		bdo_update_battery(bdo, data[14]);
	if (bdo->sensors_supported)
		bdo_report_motion(bdo, data, size);

	return 1;
}

static void bdo_rumble_worker(struct work_struct *work)
{
	struct bdo_device *bdo = container_of(work, struct bdo_device,
					      rumble_work);
	unsigned long flags;
	u8 data[5] = { BDO_OUTPUT_REPORT_ID };
	int ret;

	spin_lock_irqsave(&bdo->lock, flags);
	if (bdo->removed) {
		spin_unlock_irqrestore(&bdo->lock, flags);
		return;
	}
	data[1] = bdo->rumble_strong;
	data[2] = bdo->rumble_weak;
	spin_unlock_irqrestore(&bdo->lock, flags);

	ret = hid_hw_output_report(bdo->hdev, data, sizeof(data));
	if (ret < 0)
		hid_warn_ratelimited(bdo->hdev, "failed to send rumble report: %d\n",
				     ret);
}

static int bdo_play_effect(struct input_dev *input, void *data,
			   struct ff_effect *effect)
{
	struct bdo_device *bdo = input_get_drvdata(input);
	unsigned long flags;

	if (effect->type != FF_RUMBLE)
		return 0;

	spin_lock_irqsave(&bdo->lock, flags);
	if (bdo->removed) {
		spin_unlock_irqrestore(&bdo->lock, flags);
		return -ENODEV;
	}
	bdo->rumble_strong = effect->u.rumble.strong_magnitude >> 8;
	bdo->rumble_weak = effect->u.rumble.weak_magnitude >> 8;
	spin_unlock_irqrestore(&bdo->lock, flags);
	schedule_work(&bdo->rumble_work);

	return 0;
}

static void bdo_set_input_identity(struct bdo_device *bdo,
				   struct input_dev *input)
{
	struct hid_device *hdev = bdo->hdev;

	input->id.bustype = hdev->bus;
	input->id.vendor = hdev->vendor;
	input->id.product = hdev->product;
	/*
	 * Preserve VID/PID so SDL can associate and mute the evdev device.
	 * 0x8bd0 cannot be mistaken for a production BCD device revision.
	 */
	input->id.version = BDO_INPUT_VERSION;
	input->uniq = bdo->uniq;
	input->phys = hdev->phys;
	input_set_drvdata(input, bdo);
}

static int bdo_create_gamepad(struct bdo_device *bdo)
{
	struct input_dev *gamepad;
	int ret;

	gamepad = devm_input_allocate_device(&bdo->hdev->dev);
	if (!gamepad)
		return -ENOMEM;

	gamepad->name = devm_kstrdup(&bdo->hdev->dev, bdo->hdev->name,
				    GFP_KERNEL);
	if (!gamepad->name)
		return -ENOMEM;
	bdo_set_input_identity(bdo, gamepad);

	input_set_abs_params(gamepad, ABS_X, S16_MIN, S16_MAX, 16, 128);
	input_set_abs_params(gamepad, ABS_Y, S16_MIN, S16_MAX, 16, 128);
	input_set_abs_params(gamepad, ABS_RX, S16_MIN, S16_MAX, 16, 128);
	input_set_abs_params(gamepad, ABS_RY, S16_MIN, S16_MAX, 16, 128);
	input_set_abs_params(gamepad, ABS_Z, 0, 255, 0, 0);
	input_set_abs_params(gamepad, ABS_RZ, 0, 255, 0, 0);
	input_set_abs_params(gamepad, ABS_HAT0X, -1, 1, 0, 0);
	input_set_abs_params(gamepad, ABS_HAT0Y, -1, 1, 0, 0);

	input_set_capability(gamepad, EV_KEY, BTN_A);
	input_set_capability(gamepad, EV_KEY, BTN_B);
	input_set_capability(gamepad, EV_KEY, BTN_X);
	input_set_capability(gamepad, EV_KEY, BTN_Y);
	input_set_capability(gamepad, EV_KEY, BTN_TL);
	input_set_capability(gamepad, EV_KEY, BTN_TR);
	input_set_capability(gamepad, EV_KEY, BTN_SELECT);
	input_set_capability(gamepad, EV_KEY, BTN_START);
	input_set_capability(gamepad, EV_KEY, BTN_MODE);
	input_set_capability(gamepad, EV_KEY, BTN_THUMBL);
	input_set_capability(gamepad, EV_KEY, BTN_THUMBR);
	if (bdo->model != BDO_SF30_SN30) {
		input_set_capability(gamepad, EV_KEY, BTN_GRIPL);
		input_set_capability(gamepad, EV_KEY, BTN_GRIPR);
		input_set_capability(gamepad, EV_KEY, BTN_GRIPL2);
		input_set_capability(gamepad, EV_KEY, BTN_GRIPR2);
	}
	if (bdo->model == BDO_ULTIMATE3)
		input_set_capability(gamepad, EV_KEY, KEY_RECORD);

	if (bdo->rumble_supported) {
		input_set_capability(gamepad, EV_FF, FF_RUMBLE);
		ret = input_ff_create_memless(gamepad, NULL, bdo_play_effect);
		if (ret)
			return ret;
	}

	ret = input_register_device(gamepad);
	if (ret)
		return ret;

	bdo->gamepad = gamepad;
	return 0;
}

static int bdo_create_motion(struct bdo_device *bdo)
{
	struct input_dev *motion;
	int ret;

	if (!bdo->sensors_supported)
		return 0;

	motion = devm_input_allocate_device(&bdo->hdev->dev);
	if (!motion)
		return -ENOMEM;

	motion->name = devm_kasprintf(&bdo->hdev->dev, GFP_KERNEL,
				      "%s Motion Sensors",
				      bdo->hdev->name);
	if (!motion->name)
		return -ENOMEM;
	bdo_set_input_identity(bdo, motion);
	__set_bit(INPUT_PROP_ACCELEROMETER, motion->propbit);
	input_set_capability(motion, EV_MSC, MSC_TIMESTAMP);

	input_set_abs_params(motion, ABS_X, -BDO_ACCEL_RANGE,
			     BDO_ACCEL_RANGE, 16, 0);
	input_set_abs_params(motion, ABS_Y, -BDO_ACCEL_RANGE,
			     BDO_ACCEL_RANGE, 16, 0);
	input_set_abs_params(motion, ABS_Z, -BDO_ACCEL_RANGE,
			     BDO_ACCEL_RANGE, 16, 0);
	input_abs_set_res(motion, ABS_X, BDO_ACCEL_RES_PER_G);
	input_abs_set_res(motion, ABS_Y, BDO_ACCEL_RES_PER_G);
	input_abs_set_res(motion, ABS_Z, BDO_ACCEL_RES_PER_G);

	input_set_abs_params(motion, ABS_RX, -BDO_GYRO_RANGE,
			     BDO_GYRO_RANGE, 16, 0);
	input_set_abs_params(motion, ABS_RY, -BDO_GYRO_RANGE,
			     BDO_GYRO_RANGE, 16, 0);
	input_set_abs_params(motion, ABS_RZ, -BDO_GYRO_RANGE,
			     BDO_GYRO_RANGE, 16, 0);
	input_abs_set_res(motion, ABS_RX, BDO_GYRO_RES_PER_DPS);
	input_abs_set_res(motion, ABS_RY, BDO_GYRO_RES_PER_DPS);
	input_abs_set_res(motion, ABS_RZ, BDO_GYRO_RES_PER_DPS);

	ret = input_register_device(motion);
	if (ret)
		return ret;

	bdo->motion = motion;
	return 0;
}

static enum power_supply_property bdo_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
};

static int bdo_battery_get_property(struct power_supply *battery,
				    enum power_supply_property property,
				    union power_supply_propval *value)
{
	struct bdo_device *bdo = power_supply_get_drvdata(battery);
	unsigned long flags;

	spin_lock_irqsave(&bdo->lock, flags);
	switch (property) {
	case POWER_SUPPLY_PROP_STATUS:
		value->intval = bdo->battery_status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		value->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		value->intval = bdo->battery_capacity;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		value->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	default:
		spin_unlock_irqrestore(&bdo->lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&bdo->lock, flags);

	return 0;
}

static int bdo_create_battery(struct bdo_device *bdo)
{
	struct power_supply_config config = { .drv_data = bdo };
	int ret;

	if (!bdo->power_supported)
		return 0;

	bdo->battery_desc.name = devm_kasprintf(&bdo->hdev->dev, GFP_KERNEL,
						"8bitdo-controller-battery-%s",
						dev_name(&bdo->hdev->dev));
	if (!bdo->battery_desc.name)
		return -ENOMEM;
	bdo->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	bdo->battery_desc.properties = bdo_battery_properties;
	bdo->battery_desc.num_properties = ARRAY_SIZE(bdo_battery_properties);
	bdo->battery_desc.get_property = bdo_battery_get_property;

	bdo->battery = devm_power_supply_register(&bdo->hdev->dev,
						  &bdo->battery_desc, &config);
	if (IS_ERR(bdo->battery)) {
		ret = PTR_ERR(bdo->battery);
		bdo->battery = NULL;
		return ret;
	}

	return power_supply_powers(bdo->battery, &bdo->hdev->dev);
}

static int bdo_get_feature(struct bdo_device *bdo, u8 report_id, u8 *data)
{
	int attempt;
	int ret = -EIO;

	for (attempt = 0; attempt < BDO_FEATURE_RETRIES; attempt++) {
		memset(data, 0, BDO_FEATURE_REPORT_SIZE);
		data[0] = report_id;
		ret = hid_hw_raw_request(bdo->hdev, report_id, data,
					 BDO_FEATURE_REPORT_SIZE,
					 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
		if (ret > 0)
			return ret;
		msleep(BDO_FEATURE_RETRY_MS);
	}

	return ret;
}

static void bdo_set_feature_uniq(struct bdo_device *bdo, const u8 *data,
				 int offset, int size)
{
	if (bdo->uniq || size < offset + 6 || !data[offset + 5])
		return;

	bdo->uniq = devm_kasprintf(&bdo->hdev->dev, GFP_KERNEL,
				   "%02x:%02x:%02x:%02x:%02x:%02x",
				   data[offset + 5], data[offset + 4],
				   data[offset + 3], data[offset + 2],
				   data[offset + 1], data[offset]);
}

static int bdo_negotiate(struct bdo_device *bdo)
{
	u8 data[BDO_FEATURE_REPORT_SIZE];
	int ret;

	bdo->timestamp_initialized = false;
	bdo->sensor_timestamp_us = 0;

	switch (bdo->model) {
	case BDO_ULTIMATE2:
		return 0;
	case BDO_ULTIMATE3:
		ret = bdo_get_feature(bdo, BDO_FEATURE_REPORT_ID, data);
		if (ret < 0)
			return ret;
		bdo->rumble_supported = data[3] != 0;
		bdo->sensors_supported = data[4] != 0;
		bdo->power_supported = true;
		bdo->sensor_timestamp_supported = bdo->sensors_supported;
		bdo_set_feature_uniq(bdo, data, 11, ret);
		break;
	default:
		ret = bdo_get_feature(bdo, BDO_ENABLE_SDL_REPORT_ID, data);
		if (ret < 0)
			return ret;
		bdo->rumble_supported = true;
		bdo->sensors_supported = true;
		bdo->power_supported = true;
		bdo->sensor_timestamp_supported =
			ret >= 14 && data[13] == BDO_SENSOR_TIMESTAMP_ENABLED;
		bdo_set_feature_uniq(bdo, data, 5, ret);
		break;
	}

	bdo->sensor_interval_us = bdo_sensor_interval_us(bdo);
	return 0;
}

static int bdo_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct bdo_device *bdo;
	unsigned long timeout;
	unsigned long wait_jiffies;
	int ret;

	bdo = devm_kzalloc(&hdev->dev, sizeof(*bdo), GFP_KERNEL);
	if (!bdo)
		return -ENOMEM;

	bdo->hdev = hdev;
	bdo->model = id->driver_data;
	bdo->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
	spin_lock_init(&bdo->lock);
	init_completion(&bdo->first_report);
	INIT_WORK(&bdo->rumble_work, bdo_rumble_worker);
	hid_set_drvdata(hdev, bdo);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "failed to parse report descriptor: %d\n", ret);
		return ret;
	}
	if (!bdo_is_gamepad(hdev))
		return -ENODEV;

	/* Supply input ourselves, but leave hidraw available to HIDAPI users. */
	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW | HID_CONNECT_DRIVER);
	if (ret) {
		hid_err(hdev, "failed to start HID device: %d\n", ret);
		return ret;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "failed to open HID device: %d\n", ret);
		goto err_stop;
	}

	ret = bdo_negotiate(bdo);
	if (ret)
		hid_warn(hdev, "SDL mode negotiation failed, using legacy input: %d\n",
			 ret);

	if (bdo->model == BDO_ULTIMATE2) {
		wait_jiffies = msecs_to_jiffies(BDO_ULTIMATE2_WAIT_MS);
		timeout = wait_for_completion_timeout(&bdo->first_report, wait_jiffies);
		if (timeout && bdo->first_report_size >= BDO_ULTIMATE2_REPORT_SIZE) {
			bdo->sensors_supported = true;
			bdo->rumble_supported = true;
			bdo->power_supported = true;
		}
		bdo->sensor_interval_us = bdo_sensor_interval_us(bdo);
	}

	if (!bdo->uniq && hdev->uniq[0])
		bdo->uniq = devm_kstrdup(&hdev->dev, hdev->uniq, GFP_KERNEL);
	if (!bdo->uniq)
		bdo->uniq = devm_kstrdup(&hdev->dev, dev_name(&hdev->dev),
					 GFP_KERNEL);
	if (!bdo->uniq) {
		ret = -ENOMEM;
		goto err_close;
	}

	ret = bdo_create_gamepad(bdo);
	if (ret)
		goto err_close;
	ret = bdo_create_motion(bdo);
	if (ret)
		goto err_close;
	ret = bdo_create_battery(bdo);
	if (ret)
		goto err_close;

	hid_info(hdev, "registered SDL-mode controller\n");
	return 0;

err_close:
	cancel_work_sync(&bdo->rumble_work);
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void bdo_remove(struct hid_device *hdev)
{
	struct bdo_device *bdo = hid_get_drvdata(hdev);
	unsigned long flags;

	spin_lock_irqsave(&bdo->lock, flags);
	bdo->removed = true;
	spin_unlock_irqrestore(&bdo->lock, flags);
	cancel_work_sync(&bdo->rumble_work);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static int __maybe_unused bdo_restore_sdl_mode(struct hid_device *hdev)
{
	struct bdo_device *bdo = hid_get_drvdata(hdev);
	int ret;

	ret = bdo_negotiate(bdo);
	if (ret)
		hid_warn(hdev, "failed to restore SDL mode after resume: %d\n", ret);

	return 0;
}

static const struct hid_device_id bdo_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_SF30_PRO),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_SF30_PRO),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_SF30_PRO_BT),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_SF30_PRO_BT),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_SN30_PRO),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_SN30_PRO),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_SN30_PRO_BT),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_SN30_PRO_BT),
	  .driver_data = BDO_SF30_SN30 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_PRO_2),
	  .driver_data = BDO_PRO },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_PRO_2),
	  .driver_data = BDO_PRO },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_PRO_2_BT),
	  .driver_data = BDO_PRO },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_PRO_2_BT),
	  .driver_data = BDO_PRO },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_PRO_3),
	  .driver_data = BDO_PRO },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_PRO_3),
	  .driver_data = BDO_PRO },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_ULTIMATE_2_WIRELESS),
	  .driver_data = BDO_ULTIMATE2 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_ULTIMATE_2_WIRELESS),
	  .driver_data = BDO_ULTIMATE2 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_ULTIMATE_3),
	  .driver_data = BDO_ULTIMATE3 },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_8BITDO,
			       USB_DEVICE_ID_8BITDO_ULTIMATE_3),
	  .driver_data = BDO_ULTIMATE3 },
	{ }
};
MODULE_DEVICE_TABLE(hid, bdo_devices);

static struct hid_driver bdo_driver = {
	.name = "8bitdo",
	.id_table = bdo_devices,
	.probe = bdo_probe,
	.remove = bdo_remove,
	.raw_event = bdo_raw_event,
	.resume = pm_ptr(bdo_restore_sdl_mode),
	.reset_resume = pm_ptr(bdo_restore_sdl_mode),
};
module_hid_driver(bdo_driver);

MODULE_DESCRIPTION("8BitDo SDL-mode controller driver");
MODULE_LICENSE("GPL");
