// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 *
 * Report battery and AC adapter status through the Samsung EC mailbox.
 * The NP750XQB ACPI ECTC, BATC and ADP1 methods describe the battery
 * register layout used here.
 */

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

#define SAMSUNG_EC_POLL_INTERVAL	(30 * HZ)

#define SAMSUNG_EC_FLAGS		0x80
#define SAMSUNG_EC_BAT_STATUS	0x84
#define SAMSUNG_EC_REMAINING	0xa0
#define SAMSUNG_EC_BAT_VOLTAGE	0xa4
#define SAMSUNG_EC_CAPACITY	0xb0
#define SAMSUNG_EC_DESIGN_VOLTAGE	0xb4
#define SAMSUNG_EC_CYCLES		0xd0

struct samsung_ec_battery_data {
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

struct samsung_ec_battery {
	struct i2c_client *client;
	struct power_supply *battery;
	struct power_supply *ac;
	struct delayed_work poll_work;
	struct mutex lock;
	struct samsung_ec_battery_data data;
};

static int samsung_ec_battery_read_byte(struct i2c_client *client, u8 reg, u8 *value)
{
	u8 select[] = { 0x40, 0x00, 0xf4, 0x80, reg };
	u8 execute[] = { 0x40, 0x00, 0xff, 0x10, 0x88 };
	u8 request[] = { 0x30, 0x00, 0xf4, 0x80 };
	u8 response[2];
	struct i2c_msg msg[] = {
		{ .addr = client->addr, .len = sizeof(request), .buf = request },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = sizeof(response), .buf = response },
	};
	int ret;

	ret = i2c_master_send(client, select, sizeof(select));
	if (ret != sizeof(select))
		return ret < 0 ? ret : -EIO;

	usleep_range(5000, 6000);

	ret = i2c_master_send(client, execute, sizeof(execute));
	if (ret != sizeof(execute))
		return ret < 0 ? ret : -EIO;

	usleep_range(5000, 6000);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg))
		return ret < 0 ? ret : -EIO;
	if (response[0] != 0x50)
		return -EIO;

	*value = response[1];
	return 0;
}

static int samsung_ec_battery_read_word(struct i2c_client *client, u8 reg,
					unsigned int offset, int *value)
{
	u8 low, high;
	int ret;

	ret = samsung_ec_battery_read_byte(client, reg + offset, &high);
	if (ret)
		return ret;
	ret = samsung_ec_battery_read_byte(client, reg + offset + 1, &low);
	if (ret)
		return ret;

	*value = (high << 8) | low;
	return 0;
}

static int samsung_ec_battery_refresh(struct samsung_ec_battery *ec,
				      struct samsung_ec_battery_data *data)
{
	struct i2c_client *client = ec->client;
	u8 flags, state;
	int current_ma;
	int ret;

	ret = samsung_ec_battery_read_byte(client, SAMSUNG_EC_FLAGS, &flags);
	if (ret)
		return ret;
	ret = samsung_ec_battery_read_byte(client, SAMSUNG_EC_BAT_STATUS, &state);
	if (ret)
		return ret;

	data->present = !!(flags & BIT(0));
	data->ac_online = !!(flags & BIT(2));

	if (state & BIT(3))
		data->status = POWER_SUPPLY_STATUS_FULL;
	else if (state & BIT(1))
		data->status = POWER_SUPPLY_STATUS_CHARGING;
	else if (state & BIT(0))
		data->status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		data->status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	/* BATC._BST reads the upper big-endian word of B1RR. */
	ret = samsung_ec_battery_read_word(client, SAMSUNG_EC_REMAINING, 2,
					   &data->charge_now);
	if (ret)
		return ret;
	ret = samsung_ec_battery_read_word(client, SAMSUNG_EC_BAT_VOLTAGE, 2,
					   &data->voltage_now);
	if (ret)
		return ret;
	ret = samsung_ec_battery_read_word(client, SAMSUNG_EC_BAT_VOLTAGE, 0,
					   &current_ma);
	if (ret)
		return ret;
	/* BATC._BIX reads design capacity from the lower word of B1AF. */
	ret = samsung_ec_battery_read_word(client, SAMSUNG_EC_CAPACITY, 0,
					   &data->charge_full_design);
	if (ret)
		return ret;
	ret = samsung_ec_battery_read_word(client, SAMSUNG_EC_CAPACITY, 2,
					   &data->charge_full);
	if (ret)
		return ret;
	ret = samsung_ec_battery_read_word(client, SAMSUNG_EC_DESIGN_VOLTAGE, 0,
					   &data->voltage_min_design);
	if (ret)
		return ret;
	ret = samsung_ec_battery_read_word(client, SAMSUNG_EC_CYCLES, 0,
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

static enum power_supply_property samsung_ec_battery_properties[] = {
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
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int samsung_ec_battery_get_property(struct power_supply *psy,
					   enum power_supply_property prop,
					   union power_supply_propval *val)
{
	struct samsung_ec_battery *ec = power_supply_get_drvdata(psy);
	const struct samsung_ec_battery_data *data = &ec->data;
	int ret = 0;

	guard(mutex)(&ec->lock);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = data->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = data->present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (data->charge_now < 0 || data->charge_full <= 0)
			ret = -ENODATA;
		else
			val->intval = clamp(100 * (data->charge_now / 1000) /
					     (data->charge_full / 1000),
					     0, 100);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = data->charge_now;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = data->charge_full;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = data->charge_full_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = data->voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = data->voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = data->current_now;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = data->cycle_count;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "Galaxy Book4 Edge Battery";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Samsung";
		break;
	default:
		ret = -EINVAL;
	}

	if (!ret && (prop == POWER_SUPPLY_PROP_CHARGE_NOW ||
		     prop == POWER_SUPPLY_PROP_CHARGE_FULL ||
		     prop == POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN ||
		     prop == POWER_SUPPLY_PROP_VOLTAGE_NOW ||
		     prop == POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN ||
		     prop == POWER_SUPPLY_PROP_CURRENT_NOW ||
		     prop == POWER_SUPPLY_PROP_CYCLE_COUNT) && val->intval < 0 &&
	    !(prop == POWER_SUPPLY_PROP_CURRENT_NOW && val->intval != -1))
		ret = -ENODATA;

	return ret;
}

static int samsung_ec_ac_get_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      union power_supply_propval *val)
{
	struct samsung_ec_battery *ec = power_supply_get_drvdata(psy);

	if (prop != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	guard(mutex)(&ec->lock);
	val->intval = ec->data.ac_online;
	return 0;
}

static enum power_supply_property samsung_ec_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc samsung_ec_battery_desc = {
	.name = "samsung-galaxybook-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = samsung_ec_battery_properties,
	.num_properties = ARRAY_SIZE(samsung_ec_battery_properties),
	.get_property = samsung_ec_battery_get_property,
};

static const struct power_supply_desc samsung_ec_ac_desc = {
	.name = "samsung-galaxybook-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = samsung_ec_ac_properties,
	.num_properties = ARRAY_SIZE(samsung_ec_ac_properties),
	.get_property = samsung_ec_ac_get_property,
};

static void samsung_ec_battery_poll(struct work_struct *work)
{
	struct samsung_ec_battery *ec = container_of(to_delayed_work(work),
						struct samsung_ec_battery, poll_work);
	struct samsung_ec_battery_data data = {};
	bool changed = false;
	int ret;

	mutex_lock(&ec->lock);
	ret = samsung_ec_battery_refresh(ec, &data);
	if (ret) {
		dev_warn_ratelimited(&ec->client->dev, "EC read failed: %d\n", ret);
	} else {
		changed = memcmp(&ec->data, &data, sizeof(data));
		ec->data = data;
	}
	mutex_unlock(&ec->lock);

	if (!ret && changed) {
		power_supply_changed(ec->battery);
		power_supply_changed(ec->ac);
	}

	schedule_delayed_work(&ec->poll_work, SAMSUNG_EC_POLL_INTERVAL);
}

static int samsung_ec_battery_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct samsung_ec_battery *ec;
	int ret;

	ec = devm_kzalloc(&client->dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->client = client;
	mutex_init(&ec->lock);
	INIT_DELAYED_WORK(&ec->poll_work, samsung_ec_battery_poll);

	ret = samsung_ec_battery_refresh(ec, &ec->data);
	if (ret)
		return dev_err_probe(&client->dev, ret, "failed to read EC battery\n");

	cfg.drv_data = ec;
	cfg.fwnode = dev_fwnode(&client->dev);
	ec->battery = devm_power_supply_register(&client->dev,
						  &samsung_ec_battery_desc, &cfg);
	if (IS_ERR(ec->battery))
		return dev_err_probe(&client->dev, PTR_ERR(ec->battery),
				     "failed to register battery\n");

	ec->ac = devm_power_supply_register(&client->dev, &samsung_ec_ac_desc,
					     &cfg);
	if (IS_ERR(ec->ac))
		return dev_err_probe(&client->dev, PTR_ERR(ec->ac),
				     "failed to register AC supply\n");

	i2c_set_clientdata(client, ec);
	schedule_delayed_work(&ec->poll_work, SAMSUNG_EC_POLL_INTERVAL);

	return 0;
}

static void samsung_ec_battery_remove(struct i2c_client *client)
{
	struct samsung_ec_battery *ec = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&ec->poll_work);
}

static const struct of_device_id samsung_ec_battery_of_match[] = {
	{ .compatible = "samsung,galaxybook-ec-battery" },
	{}
};
MODULE_DEVICE_TABLE(of, samsung_ec_battery_of_match);

static const struct i2c_device_id samsung_ec_battery_i2c_id[] = {
	{ "galaxybook-ec-bat" },
	{}
};
MODULE_DEVICE_TABLE(i2c, samsung_ec_battery_i2c_id);

static struct i2c_driver samsung_ec_battery_driver = {
	.driver = {
		.name = "samsung-galaxybook-ec-battery",
		.of_match_table = samsung_ec_battery_of_match,
	},
	.probe = samsung_ec_battery_probe,
	.remove = samsung_ec_battery_remove,
	.id_table = samsung_ec_battery_i2c_id,
};
module_i2c_driver(samsung_ec_battery_driver);

MODULE_DESCRIPTION("Samsung Galaxy Book4 Edge EC battery");
MODULE_LICENSE("GPL");
