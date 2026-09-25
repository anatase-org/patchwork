// SPDX-License-Identifier: GPL-2.0
/*
 * Zotac ZONE fan control and monitoring
 *
 * This driver references the addresses and reverse engineering of Luke D.
 * Jones as part of his work for an OOT Zotac Zone platform driver. This
 * is a re-implementation that ports only the hwmon EC functionality using
 * a standard auto/manual control to align with other hwmon handheld
 * drivers such as oxp-ec, ayaneo-ec and gpd-fan (the original driver
 * implemented a novel kernel managed curve).
 *
 * Copyright (C) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 */

#include <linux/cleanup.h>
#include <linux/dmi.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define ZOTAC_EC_CMD_PORT		0x4e
#define ZOTAC_EC_DATA_PORT		0x4f

#define ZOTAC_EC_FAN_MODE		0x44a
#define ZOTAC_EC_FAN_PWM		0x44b
#define ZOTAC_EC_TEMP			0x462
#define ZOTAC_EC_FAN_RPM_HIGH		0x476
#define ZOTAC_EC_FAN_RPM_LOW		0x477

#define ZOTAC_EC_MODE_AUTO		0
#define ZOTAC_EC_MODE_MANUAL		1

struct zotac_zone_data {
	struct mutex lock;
};

/* The caller holds data->lock across the complete EC transaction. */
static void zotac_zone_select(u16 addr)
{
	outb(0x2e, ZOTAC_EC_CMD_PORT);
	outb(0x11, ZOTAC_EC_DATA_PORT);
	outb(0x2f, ZOTAC_EC_CMD_PORT);
	outb(addr >> 8, ZOTAC_EC_DATA_PORT);

	outb(0x2e, ZOTAC_EC_CMD_PORT);
	outb(0x10, ZOTAC_EC_DATA_PORT);
	outb(0x2f, ZOTAC_EC_CMD_PORT);
	outb(addr, ZOTAC_EC_DATA_PORT);

	outb(0x2e, ZOTAC_EC_CMD_PORT);
	outb(0x12, ZOTAC_EC_DATA_PORT);
	outb(0x2f, ZOTAC_EC_CMD_PORT);
}

static u8 zotac_zone_read(u16 addr)
{
	zotac_zone_select(addr);
	return inb(ZOTAC_EC_DATA_PORT);
}

static void zotac_zone_write(u16 addr, u8 value)
{
	zotac_zone_select(addr);
	outb(value, ZOTAC_EC_DATA_PORT);
}

static umode_t zotac_zone_is_visible(const void *drvdata,
				     enum hwmon_sensor_types type, u32 attr,
				     int channel)
{
	if (channel)
		return 0;

	switch (type) {
	case hwmon_fan:
		return attr == hwmon_fan_input ? 0444 : 0;
	case hwmon_temp:
		return attr == hwmon_temp_input ? 0444 : 0;
	case hwmon_pwm:
		return attr == hwmon_pwm_input || attr == hwmon_pwm_enable ?
			0644 : 0;
	default:
		return 0;
	}
}

static int zotac_zone_hwmon_read(struct device *dev,
				 enum hwmon_sensor_types type, u32 attr,
				 int channel, long *val)
{
	struct zotac_zone_data *data = dev_get_drvdata(dev);
	u8 mode;
	int ret = 0;

	if (channel)
		return -EOPNOTSUPP;

	guard(mutex)(&data->lock);
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input) {
			*val = zotac_zone_read(ZOTAC_EC_FAN_RPM_HIGH) << 8;
			*val |= zotac_zone_read(ZOTAC_EC_FAN_RPM_LOW);
		} else {
			ret = -EOPNOTSUPP;
		}
		break;
	case hwmon_temp:
		if (attr == hwmon_temp_input)
			*val = zotac_zone_read(ZOTAC_EC_TEMP) * 1000;
		else
			ret = -EOPNOTSUPP;
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			*val = zotac_zone_read(ZOTAC_EC_FAN_PWM);
			break;
		case hwmon_pwm_enable:
			mode = zotac_zone_read(ZOTAC_EC_FAN_MODE);
			if (mode == ZOTAC_EC_MODE_AUTO)
				*val = 2;
			else if (mode == ZOTAC_EC_MODE_MANUAL)
				*val = 1;
			else
				ret = -EIO;
			break;
		default:
			ret = -EOPNOTSUPP;
		}
		break;
	default:
		ret = -EOPNOTSUPP;
	}
	return ret;
}

static int zotac_zone_hwmon_write(struct device *dev,
				  enum hwmon_sensor_types type, u32 attr,
				  int channel, long val)
{
	struct zotac_zone_data *data = dev_get_drvdata(dev);
	u16 reg;
	u8 value;

	if (type != hwmon_pwm || channel)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_pwm_input:
		if (val < 0 || val > 255)
			return -EINVAL;
		reg = ZOTAC_EC_FAN_PWM;
		value = val;
		break;
	case hwmon_pwm_enable:
		if (val != 1 && val != 2)
			return -EINVAL;
		reg = ZOTAC_EC_FAN_MODE;
		value = val == 1 ? ZOTAC_EC_MODE_MANUAL : ZOTAC_EC_MODE_AUTO;
		break;
	default:
		return -EOPNOTSUPP;
	}

	guard(mutex)(&data->lock);
	zotac_zone_write(reg, value);

	return 0;
}

static const struct hwmon_ops zotac_zone_hwmon_ops = {
	.is_visible = zotac_zone_is_visible,
	.read = zotac_zone_hwmon_read,
	.write = zotac_zone_hwmon_write,
};

static const struct hwmon_channel_info * const zotac_zone_hwmon_channels[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL,
};

static const struct hwmon_chip_info zotac_zone_hwmon_chip_info = {
	.ops = &zotac_zone_hwmon_ops,
	.info = zotac_zone_hwmon_channels,
};

static int zotac_zone_probe(struct platform_device *pdev)
{
	struct zotac_zone_data *data;
	struct device *hwmon_dev;
	int ret;

	if (!devm_request_region(&pdev->dev, ZOTAC_EC_CMD_PORT, 2,
				 "zotac-zone-ec"))
		return -EBUSY;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = devm_mutex_init(&pdev->dev, &data->lock);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, data);
	hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
						   "zotac_zone_ec", data,
						   &zotac_zone_hwmon_chip_info,
						   NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static void zotac_zone_remove(struct platform_device *pdev)
{
	struct zotac_zone_data *data = platform_get_drvdata(pdev);

	guard(mutex)(&data->lock);
	zotac_zone_write(ZOTAC_EC_FAN_MODE, ZOTAC_EC_MODE_AUTO);
}

static struct platform_driver zotac_zone_driver = {
	.driver = {
		.name = "zotac-zone-ec",
	},
	.probe = zotac_zone_probe,
	.remove = zotac_zone_remove,
};

static const struct dmi_system_id zotac_zone_dmi_table[] = {
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "ZOTAC"),
			DMI_MATCH(DMI_BOARD_NAME, "G0A1W"),
		},
	},
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "ZOTAC"),
			DMI_MATCH(DMI_PRODUCT_NAME, "ZOTAC GAMING ZONE"),
		},
	},
	{ }
};
MODULE_DEVICE_TABLE(dmi, zotac_zone_dmi_table);

static struct platform_device *zotac_zone_device;

static int __init zotac_zone_init(void)
{
	if (!dmi_check_system(zotac_zone_dmi_table))
		return -ENODEV;

	zotac_zone_device = platform_create_bundle(&zotac_zone_driver,
					     zotac_zone_probe, NULL, 0, NULL, 0);

	return PTR_ERR_OR_ZERO(zotac_zone_device);
}

static void __exit zotac_zone_exit(void)
{
	platform_device_unregister(zotac_zone_device);
	platform_driver_unregister(&zotac_zone_driver);
}

module_init(zotac_zone_init);
module_exit(zotac_zone_exit);

MODULE_AUTHOR("Antheas Kapenekakis <lkml@antheas.dev>");
MODULE_DESCRIPTION("Zotac ZONE fan hwmon driver");
MODULE_LICENSE("GPL");
