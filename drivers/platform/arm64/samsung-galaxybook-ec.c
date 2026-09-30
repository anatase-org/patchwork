// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
/*
 * Copyright (c) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 *
 * Report battery and AC adapter status and control the keyboard backlight
 * through the ENE KB9058 EC mailbox, and report EC hotkey events.
 * The NP750XQB ACPI ECTC, BATC and ADP1 methods describe the battery
 * register layout used here.
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/platform_profile.h>
#include <linux/slab.h>
#include <linux/string.h>

#define GALAXYBOOK_EC_FLAGS		0x80
#define GALAXYBOOK_EC_BAT_STATUS	0x84
#define GALAXYBOOK_EC_REMAINING		0xa0
#define GALAXYBOOK_EC_BAT_VOLTAGE	0xa4
#define GALAXYBOOK_EC_CAPACITY		0xb0
#define GALAXYBOOK_EC_DESIGN_VOLTAGE	0xb4
#define GALAXYBOOK_EC_CYCLES		0xd0

#define GALAXYBOOK_EC_BAT_PRESENT	BIT(0)
#define GALAXYBOOK_EC_AC_PRESENT	BIT(2)
#define GALAXYBOOK_EC_BAT_DISCHARGING	BIT(0)
#define GALAXYBOOK_EC_BAT_CHARGING	BIT(1)
#define GALAXYBOOK_EC_BAT_FULL		BIT(3)

struct galaxybook_battery_data {
	int status;
	int present;
	int ac_online;
	int charge_now;
	int charge_full;
	int charge_full_design;
	int voltage_now;
	int voltage_min_design;
	int current_now;
	int cycle_count;
};

struct samsung_galaxybook_ec {
	struct i2c_client *client;
	struct i2c_client *events;
	struct power_supply *battery;
	struct power_supply *ac;
	struct led_classdev kbd_backlight;
	struct device *profile_dev;
	enum platform_profile_option profile;
	/* Serializes mailbox transactions and cached supply/profile state. */
	struct mutex lock;
	struct galaxybook_battery_data data;
};

static int galaxybook_ec_read_mailbox(struct i2c_client *client, u16 address,
				     u8 *value)
{
	u8 request[] = { 0x30, 0x00, address >> 8, address & 0xff };
	u8 response[2];
	struct i2c_msg msg[] = {
		{ .addr = client->addr, .len = sizeof(request), .buf = request },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = sizeof(response), .buf = response },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg))
		return ret < 0 ? ret : -EIO;
	if (response[0] != 0x50)
		return -EIO;

	*value = response[1];
	return 0;
}

static int galaxybook_ec_wait_ready(struct i2c_client *client)
{
	u8 command;
	int ret, i;

	/* Wait for the command register to clear. */
	for (i = 0; i < 30; i++) {
		ret = galaxybook_ec_read_mailbox(client, 0xff10, &command);
		if (ret)
			return ret;
		if (!command)
			return 0;

		usleep_range(1000, 2000);
	}

	return -ETIMEDOUT;
}

static int galaxybook_kbd_backlight_set(struct led_classdev *led,
				      enum led_brightness brightness)
{
	struct samsung_galaxybook_ec *ec =
		container_of(led, struct samsung_galaxybook_ec, kbd_backlight);
	u8 select[] = { 0x40, 0x00, 0xf4, 0x80, brightness };
	u8 execute[] = { 0x40, 0x00, 0xff, 0x10, 0xfd };
	int ret;

	guard(mutex)(&ec->lock);

	ret = galaxybook_ec_wait_ready(ec->client);
	if (ret)
		return ret;

	ret = i2c_master_send(ec->client, select, sizeof(select));
	if (ret != sizeof(select))
		return ret < 0 ? ret : -EIO;

	usleep_range(5000, 6000);

	ret = i2c_master_send(ec->client, execute, sizeof(execute));
	if (ret != sizeof(execute))
		return ret < 0 ? ret : -EIO;

	return galaxybook_ec_wait_ready(ec->client);
}

static int galaxybook_kbd_backlight_read(struct samsung_galaxybook_ec *ec,
				       enum led_brightness *brightness)
{
	u8 select[] = { 0x40, 0x00, 0xf4, 0x80, 0x00 };
	u8 execute[] = { 0x40, 0x00, 0xff, 0x10, 0xfc };
	u8 level;
	int ret;

	guard(mutex)(&ec->lock);

	ret = galaxybook_ec_wait_ready(ec->client);
	if (ret)
		return ret;

	/* Stage two zero bytes before executing the read command. */
	ret = i2c_master_send(ec->client, select, sizeof(select));
	if (ret != sizeof(select))
		return ret < 0 ? ret : -EIO;

	usleep_range(5000, 6000);

	select[3] = 0x81;
	ret = i2c_master_send(ec->client, select, sizeof(select));
	if (ret != sizeof(select))
		return ret < 0 ? ret : -EIO;

	usleep_range(5000, 6000);

	ret = i2c_master_send(ec->client, execute, sizeof(execute));
	if (ret != sizeof(execute))
		return ret < 0 ? ret : -EIO;

	ret = galaxybook_ec_wait_ready(ec->client);
	if (ret)
		return ret;

	ret = galaxybook_ec_read_mailbox(ec->client, 0xf480, &level);
	if (ret)
		return ret;
	if (level > ec->kbd_backlight.max_brightness)
		return -EIO;

	*brightness = level;
	return 0;
}

static enum led_brightness galaxybook_kbd_backlight_get(struct led_classdev *led)
{
	struct samsung_galaxybook_ec *ec =
		container_of(led, struct samsung_galaxybook_ec, kbd_backlight);
	enum led_brightness brightness;
	int ret;

	ret = galaxybook_kbd_backlight_read(ec, &brightness);
	if (ret) {
		dev_warn_ratelimited(&ec->client->dev,
				     "failed to read keyboard backlight: %d\n", ret);
		return -EIO;
	}

	return brightness;
}

static int galaxybook_ec_read_byte(struct i2c_client *client, u8 reg, u8 *value)
{
	u8 select[] = { 0x40, 0x00, 0xf4, 0x80, reg };
	u8 execute[] = { 0x40, 0x00, 0xff, 0x10, 0x88 };
	int ret;

	ret = i2c_master_send(client, select, sizeof(select));
	if (ret != sizeof(select))
		return ret < 0 ? ret : -EIO;

	usleep_range(5000, 6000);

	ret = i2c_master_send(client, execute, sizeof(execute));
	if (ret != sizeof(execute))
		return ret < 0 ? ret : -EIO;

	usleep_range(5000, 6000);

	return galaxybook_ec_read_mailbox(client, 0xf480, value);
}

static int galaxybook_ec_read_word(struct i2c_client *client, u8 reg,
				   unsigned int offset, int *value)
{
	u8 low, high;
	int ret;

	ret = galaxybook_ec_read_byte(client, reg + offset, &high);
	if (ret)
		return ret;
	ret = galaxybook_ec_read_byte(client, reg + offset + 1, &low);
	if (ret)
		return ret;

	*value = (high << 8) | low;
	return 0;
}

static int galaxybook_battery_refresh(struct samsung_galaxybook_ec *ec,
				      struct galaxybook_battery_data *data)
{
	struct i2c_client *client = ec->client;
	u8 flags, state;
	int current_ma;
	int ret;

	ret = galaxybook_ec_read_byte(client, GALAXYBOOK_EC_FLAGS, &flags);
	if (ret)
		return ret;
	ret = galaxybook_ec_read_byte(client, GALAXYBOOK_EC_BAT_STATUS, &state);
	if (ret)
		return ret;

	data->present = !!(flags & GALAXYBOOK_EC_BAT_PRESENT);
	data->ac_online = !!(flags & GALAXYBOOK_EC_AC_PRESENT);

	if (state & GALAXYBOOK_EC_BAT_FULL)
		data->status = POWER_SUPPLY_STATUS_FULL;
	else if (state & GALAXYBOOK_EC_BAT_CHARGING)
		data->status = POWER_SUPPLY_STATUS_CHARGING;
	else if (state & GALAXYBOOK_EC_BAT_DISCHARGING)
		data->status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		data->status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	/* BATC._BST reads the upper big-endian word of B1RR. */
	ret = galaxybook_ec_read_word(client, GALAXYBOOK_EC_REMAINING, 2,
				      &data->charge_now);
	if (ret)
		return ret;
	ret = galaxybook_ec_read_word(client, GALAXYBOOK_EC_BAT_VOLTAGE, 2,
				      &data->voltage_now);
	if (ret)
		return ret;
	ret = galaxybook_ec_read_word(client, GALAXYBOOK_EC_BAT_VOLTAGE, 0,
				      &current_ma);
	if (ret)
		return ret;
	/* BATC._BIX reads design capacity from the lower word of B1AF. */
	ret = galaxybook_ec_read_word(client, GALAXYBOOK_EC_CAPACITY, 0,
				      &data->charge_full_design);
	if (ret)
		return ret;
	ret = galaxybook_ec_read_word(client, GALAXYBOOK_EC_CAPACITY, 2,
				      &data->charge_full);
	if (ret)
		return ret;
	ret = galaxybook_ec_read_word(client, GALAXYBOOK_EC_DESIGN_VOLTAGE, 0,
				      &data->voltage_min_design);
	if (ret)
		return ret;
	ret = galaxybook_ec_read_word(client, GALAXYBOOK_EC_CYCLES, 0,
				      &data->cycle_count);
	if (ret)
		return ret;

	/* Unknown EC values match BATC._BST/_BIX's 0xffff sentinel. */
	if (data->charge_now == 0xffff)
		data->charge_now = -1;
	if (data->charge_full_design == 0xffff)
		data->charge_full_design = -1;
	if (data->charge_full == 0xffff)
		data->charge_full = -1;
	if (data->voltage_now == 0xffff)
		data->voltage_now = -1;
	if (data->voltage_min_design == 0xffff)
		data->voltage_min_design = -1;
	if (current_ma == 0xffff)
		current_ma = -1;
	if (data->cycle_count == 0xffff)
		data->cycle_count = -1;

	if (current_ma >= 0) {
		current_ma = abs((s16)current_ma);
		if (data->status == POWER_SUPPLY_STATUS_DISCHARGING)
			current_ma = -current_ma;
	}

	data->current_now = current_ma;

	/* EC units are mAh, mV and mA; power_supply uses micro units. */
	if (data->charge_now >= 0)
		data->charge_now *= 1000;
	if (data->charge_full >= 0)
		data->charge_full *= 1000;
	if (data->charge_full_design >= 0)
		data->charge_full_design *= 1000;
	if (data->voltage_now >= 0)
		data->voltage_now *= 1000;
	if (data->voltage_min_design >= 0)
		data->voltage_min_design *= 1000;
	if (current_ma != -1)
		data->current_now *= 1000;

	return 0;
}

static enum power_supply_property galaxybook_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int galaxybook_battery_get_property(struct power_supply *psy,
					   enum power_supply_property prop,
					   union power_supply_propval *val)
{
	struct samsung_galaxybook_ec *ec = power_supply_get_drvdata(psy);
	const struct galaxybook_battery_data *data = &ec->data;
	int value;

	guard(mutex)(&ec->lock);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		value = data->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		value = data->present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		value = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (data->charge_now < 0 || data->charge_full <= 0)
			return -ENODATA;
		value = clamp(100 * (data->charge_now / 1000) /
			      (data->charge_full / 1000), 0, 100);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		value = data->charge_now;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		value = data->charge_full;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		value = data->charge_full_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		value = data->voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		value = data->voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		value = data->current_now;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		value = data->cycle_count;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "Samsung Galaxy Book Battery";
		return 0;
	default:
		return -EINVAL;
	}

	/* Negative current is valid while discharging; -1 marks unknown data. */
	if (value < 0 && (prop != POWER_SUPPLY_PROP_CURRENT_NOW || value == -1))
		return -ENODATA;

	val->intval = value;
	return 0;
}

static int galaxybook_ac_get_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      union power_supply_propval *val)
{
	struct samsung_galaxybook_ec *ec = power_supply_get_drvdata(psy);

	guard(mutex)(&ec->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ec->data.ac_online;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "Samsung Galaxy Book AC Adapter";
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property galaxybook_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static const struct power_supply_desc galaxybook_battery_desc = {
	.name = "samsung-galaxybook-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = galaxybook_battery_properties,
	.num_properties = ARRAY_SIZE(galaxybook_battery_properties),
	.get_property = galaxybook_battery_get_property,
};

static const struct power_supply_desc galaxybook_ac_desc = {
	.name = "samsung-galaxybook-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = galaxybook_ac_properties,
	.num_properties = ARRAY_SIZE(galaxybook_ac_properties),
	.get_property = galaxybook_ac_get_property,
};

static int galaxybook_profile_set(struct device *dev,
				 enum platform_profile_option profile)
{
	struct samsung_galaxybook_ec *ec = dev_get_drvdata(dev);
	u8 select[] = { 0x40, 0x00, 0xf4, 0x80, 0x80 };
	u8 execute[] = { 0x40, 0x00, 0xff, 0x10, 0xee };
	u8 mode;
	int ret;

	/* The Galaxy Book4 Edge ACPI PRF3 method maps Optimized to EC auto mode (0). */
	switch (profile) {
	case PLATFORM_PROFILE_QUIET:
		mode = 0x0a;
		break;
	case PLATFORM_PROFILE_BALANCED:
		mode = 0x00;
		break;
	case PLATFORM_PROFILE_PERFORMANCE:
		mode = 0x15;
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&ec->lock);

	ret = galaxybook_ec_wait_ready(ec->client);
	if (ret)
		return ret;

	/* PRF3 issues extended command 0xee with payload [0x80, mode]. */
	ret = i2c_master_send(ec->client, select, sizeof(select));
	if (ret != sizeof(select))
		return ret < 0 ? ret : -EIO;
	usleep_range(5000, 6000);

	select[3] = 0x81;
	select[4] = mode;
	ret = i2c_master_send(ec->client, select, sizeof(select));
	if (ret != sizeof(select))
		return ret < 0 ? ret : -EIO;
	usleep_range(5000, 6000);

	ret = i2c_master_send(ec->client, execute, sizeof(execute));
	if (ret != sizeof(execute))
		return ret < 0 ? ret : -EIO;

	ret = galaxybook_ec_wait_ready(ec->client);
	if (!ret)
		ec->profile = profile;
	return ret;
}

static int galaxybook_profile_get(struct device *dev,
				 enum platform_profile_option *profile)
{
	struct samsung_galaxybook_ec *ec = dev_get_drvdata(dev);

	guard(mutex)(&ec->lock);

	/* No EC profile getter is defined; return the last completed selection. */
	*profile = ec->profile;
	return 0;
}

static int galaxybook_profile_probe(void *drvdata, unsigned long *choices)
{
	__set_bit(PLATFORM_PROFILE_QUIET, choices);
	__set_bit(PLATFORM_PROFILE_BALANCED, choices);
	__set_bit(PLATFORM_PROFILE_PERFORMANCE, choices);
	return 0;
}

static const struct platform_profile_ops galaxybook_profile_ops = {
	.probe = galaxybook_profile_probe,
	.profile_get = galaxybook_profile_get,
	.profile_set = galaxybook_profile_set,
};

static bool galaxybook_ec_handle_event(struct samsung_galaxybook_ec *ec)
{
	u8 event[12];
	unsigned int brightness;
	int ret;

	if (!ec->events)
		return false;

	/* Read one event packet without a preceding command write. */
	ret = i2c_master_recv(ec->events, event, sizeof(event));
	if (ret != sizeof(event)) {
		dev_warn_ratelimited(&ec->client->dev,
				     "EC event read failed: %d\n",
				     ret < 0 ? ret : -EIO);
		return false;
	}

	if (event[0] != 0x01)
		return false;

	if (event[2] == 0x0b) {
		if (ec->profile_dev) {
			ret = platform_profile_cycle();
			if (ret)
				dev_warn_ratelimited(&ec->client->dev,
						     "failed to cycle platform profile: %d\n", ret);
		}
		return true;
	}

	if (event[2] == 0x09) {
		if (ec->kbd_backlight.dev) {
			brightness = (ec->kbd_backlight.brightness + 1) %
				     (ec->kbd_backlight.max_brightness + 1);
			ret = led_set_brightness_sync(&ec->kbd_backlight, brightness);
			if (ret)
				dev_warn_ratelimited(&ec->client->dev,
						     "failed to set keyboard backlight: %d\n", ret);
			else
				led_classdev_notify_brightness_hw_changed(&ec->kbd_backlight,
									 brightness);
		}
		return true;
	}

	return false;
}

static irqreturn_t galaxybook_ec_irq_thread(int irq, void *ptr)
{
	struct samsung_galaxybook_ec *ec = ptr;
	struct galaxybook_battery_data data = {};
	bool changed = false;
	int ret;

	if (galaxybook_ec_handle_event(ec))
		return IRQ_HANDLED;

	scoped_guard(mutex, &ec->lock) {
		ret = galaxybook_battery_refresh(ec, &data);
		if (ret) {
			dev_warn_ratelimited(&ec->client->dev,
					     "EC read failed: %d\n", ret);
		} else {
			changed = memcmp(&ec->data, &data, sizeof(data));
			ec->data = data;
		}
	}

	if (!ret && changed) {
		power_supply_changed(ec->battery);
		power_supply_changed(ec->ac);
	}

	return IRQ_HANDLED;
}

static int samsung_galaxybook_ec_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct samsung_galaxybook_ec *ec;
	struct device_node *node, *bus;
	struct i2c_adapter *adapter;
	u32 address;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(&client->dev, -EOPNOTSUPP,
				     "adapter lacks combined I2C transfers\n");
	if (!client->irq)
		return dev_err_probe(&client->dev, -EINVAL,
				     "EC interrupt is required\n");

	ec = devm_kzalloc(&client->dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->client = client;
	mutex_init(&ec->lock);

	ret = galaxybook_battery_refresh(ec, &ec->data);
	if (ret)
		return dev_err_probe(&client->dev, ret, "failed to read EC battery\n");

	cfg.drv_data = ec;
	cfg.fwnode = dev_fwnode(&client->dev);
	ec->battery = devm_power_supply_register(&client->dev,
						  &galaxybook_battery_desc, &cfg);
	if (IS_ERR(ec->battery))
		return dev_err_probe(&client->dev, PTR_ERR(ec->battery),
				     "failed to register battery\n");

	ec->ac = devm_power_supply_register(&client->dev, &galaxybook_ac_desc,
					     &cfg);
	if (IS_ERR(ec->ac))
		return dev_err_probe(&client->dev, PTR_ERR(ec->ac),
				     "failed to register AC supply\n");

	ec->kbd_backlight.name = "samsung-galaxybook::kbd_backlight";
	ec->kbd_backlight.max_brightness = 3;
	ec->kbd_backlight.brightness_set_blocking = galaxybook_kbd_backlight_set;
	ec->kbd_backlight.brightness_get = galaxybook_kbd_backlight_get;
	ec->kbd_backlight.flags = LED_CORE_SUSPENDRESUME | LED_BRIGHT_HW_CHANGED;

	ret = galaxybook_kbd_backlight_read(ec, &ec->kbd_backlight.brightness);
	if (!ret) {
		ret = devm_led_classdev_register(&client->dev, &ec->kbd_backlight);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to register keyboard backlight\n");
	}

	node = of_parse_phandle(client->dev.of_node, "samsung,event-interface", 0);
	if (node) {
		ret = of_property_read_u32(node, "reg", &address);
		bus = of_get_parent(node);
		of_node_put(node);
		if (ret || address > 0x7f) {
			of_node_put(bus);
			return dev_err_probe(&client->dev, -EINVAL,
					     "invalid EC event address\n");
		}

		adapter = of_get_i2c_adapter_by_node(bus);
		of_node_put(bus);
		if (!adapter)
			return dev_err_probe(&client->dev, -EPROBE_DEFER,
					     "EC event bus is not ready\n");

		if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
			i2c_put_adapter(adapter);
			return dev_err_probe(&client->dev, -EOPNOTSUPP,
					     "event adapter lacks I2C transfers\n");
		}

		ec->events = devm_i2c_new_dummy_device(&client->dev, adapter, address);
		i2c_put_adapter(adapter);
		if (IS_ERR(ec->events))
			return dev_err_probe(&client->dev, PTR_ERR(ec->events),
					     "failed to initialize EC event interface\n");
	}

	i2c_set_clientdata(client, ec);

	/* Establish a known fan policy before exposing the cached profile. */
	ret = galaxybook_profile_set(&client->dev, PLATFORM_PROFILE_BALANCED);
	if (!ret) {
		ec->profile_dev = devm_platform_profile_register(&client->dev,
							 "samsung-galaxybook", ec,
							 &galaxybook_profile_ops);
		if (IS_ERR(ec->profile_dev))
			return dev_err_probe(&client->dev, PTR_ERR(ec->profile_dev),
					     "failed to register platform profiles\n");
	} else {
		dev_warn(&client->dev, "platform profiles unavailable: %d\n", ret);
	}
	return devm_request_threaded_irq(&client->dev, client->irq, NULL,
					 galaxybook_ec_irq_thread, IRQF_ONESHOT,
					 dev_name(&client->dev), ec);
}

static const struct of_device_id samsung_galaxybook_ec_of_match[] = {
	{ .compatible = "samsung,galaxybook4-edge-ec" },
	{}
};
MODULE_DEVICE_TABLE(of, samsung_galaxybook_ec_of_match);

static struct i2c_driver samsung_galaxybook_ec_driver = {
	.driver = {
		.name = "samsung-galaxybook-ec",
		.of_match_table = samsung_galaxybook_ec_of_match,
	},
	.probe = samsung_galaxybook_ec_probe,
};
module_i2c_driver(samsung_galaxybook_ec_driver);

MODULE_DESCRIPTION("Samsung Galaxy Book embedded controller");
MODULE_LICENSE("Dual BSD/GPL");
