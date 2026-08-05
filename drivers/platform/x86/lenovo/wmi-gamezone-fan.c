// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Lenovo GameZone fan WMI interface driver.
 *
 * Copyright (C) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 */

#include <linux/acpi.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/wmi.h>

#define LENOVO_GAMEZONE_FAN_GUID "92549549-4BDE-4F06-AC04-CE8BF898DBAA"

#define LWMI_GZ_FAN_METHOD_CURVE_GET	5
#define LWMI_GZ_FAN_METHOD_CURVE_SET	6

#define LWMI_GZ_FAN_CURVE_POINTS		10
#define LWMI_GZ_FAN_MAX_PERCENT			100
#define LWMI_GZ_FAN_MAX_RAW				115

enum lwmi_gz_fan_attr_type {
	LWMI_GZ_FAN_ATTR_CURVE_PWM,
	LWMI_GZ_FAN_ATTR_CURVE_TEMP,
};

struct lwmi_gz_fan_curve {
	u32 pwm[LWMI_GZ_FAN_CURVE_POINTS];
	u32 temp[LWMI_GZ_FAN_CURVE_POINTS];
};

struct lwmi_gz_fan_curve_raw {
	__le32 pwm_count;
	__le32 pwm[LWMI_GZ_FAN_CURVE_POINTS];
	__le32 temp_count;
	__le32 temp[LWMI_GZ_FAN_CURVE_POINTS];
};

struct lwmi_gz_fan_priv {
	struct wmi_device *wdev;
	struct mutex lock; /* WMI method serialization */
};

static int lwmi_gz_fan_eval(struct lwmi_gz_fan_priv *priv, u32 method,
			    const void *data, size_t size,
			    union acpi_object **ret_obj)
{
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_buffer input = { size, (void *)data };
	acpi_status status;

	status = wmidev_evaluate_method(priv->wdev, 0, method, &input, &output);
	if (ACPI_FAILURE(status))
		return -EIO;

	if (ret_obj)
		*ret_obj = output.pointer;
	else
		kfree(output.pointer);

	return 0;
}

static int lwmi_gz_fan_curve_get(struct lwmi_gz_fan_priv *priv,
				 struct lwmi_gz_fan_curve *curve)
{
	union acpi_object *ret_obj __free(kfree) = NULL;
	struct lwmi_gz_fan_curve_raw raw;
	u32 args = 0;
	int i;
	int ret;

	ret = lwmi_gz_fan_eval(priv, LWMI_GZ_FAN_METHOD_CURVE_GET,
			       &args, sizeof(args), &ret_obj);
	if (ret)
		return ret;
	if (!ret_obj)
		return -ENODATA;
	if (ret_obj->type != ACPI_TYPE_BUFFER ||
	    ret_obj->buffer.length < sizeof(raw))
		return -ENXIO;

	memcpy(&raw, ret_obj->buffer.pointer, sizeof(raw));
	if (le32_to_cpu(raw.pwm_count) != LWMI_GZ_FAN_CURVE_POINTS ||
	    le32_to_cpu(raw.temp_count) != LWMI_GZ_FAN_CURVE_POINTS)
		return -EINVAL;

	for (i = 0; i < LWMI_GZ_FAN_CURVE_POINTS; i++) {
		curve->pwm[i] = le32_to_cpu(raw.pwm[i]);
		curve->temp[i] = le32_to_cpu(raw.temp[i]);

		if (curve->pwm[i] > LWMI_GZ_FAN_MAX_RAW ||
		    curve->temp[i] > U16_MAX)
			return -ERANGE;
	}

	return 0;
}

static int lwmi_gz_fan_curve_set(struct lwmi_gz_fan_priv *priv,
				 const struct lwmi_gz_fan_curve *curve)
{
	u8 args[52] = {};
	int i;

	/* Fan ID and sensor ID are currently ignored by the firmware. */
	put_unaligned_le32(LWMI_GZ_FAN_CURVE_POINTS, &args[2]);
	for (i = 0; i < LWMI_GZ_FAN_CURVE_POINTS; i++)
		put_unaligned_le16(curve->pwm[i], &args[6 + i * sizeof(u16)]);

	put_unaligned_le32(LWMI_GZ_FAN_CURVE_POINTS, &args[27]);
	for (i = 0; i < LWMI_GZ_FAN_CURVE_POINTS; i++)
		put_unaligned_le16(curve->temp[i], &args[31 + i * sizeof(u16)]);

	return lwmi_gz_fan_eval(priv, LWMI_GZ_FAN_METHOD_CURVE_SET,
				args, sizeof(args), NULL);
}

static ssize_t lwmi_gz_fan_curve_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct lwmi_gz_fan_priv *priv = dev_get_drvdata(dev);
	struct lwmi_gz_fan_curve curve;
	u32 value;
	int ret;

	guard(mutex)(&priv->lock);

	ret = lwmi_gz_fan_curve_get(priv, &curve);
	if (ret)
		return ret;

	if (sattr->nr == LWMI_GZ_FAN_ATTR_CURVE_PWM)
		value = DIV_ROUND_CLOSEST(min(curve.pwm[sattr->index],
					      LWMI_GZ_FAN_MAX_PERCENT) * 255,
					  LWMI_GZ_FAN_MAX_PERCENT);
	else
		value = curve.temp[sattr->index];

	return sysfs_emit(buf, "%u\n", value);
}

static ssize_t lwmi_gz_fan_curve_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	struct lwmi_gz_fan_priv *priv = dev_get_drvdata(dev);
	struct lwmi_gz_fan_curve curve;
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	if (value > 255)
		return -EINVAL;

	guard(mutex)(&priv->lock);

	ret = lwmi_gz_fan_curve_get(priv, &curve);
	if (ret)
		return ret;

	curve.pwm[sattr->index] =
		DIV_ROUND_CLOSEST(value * LWMI_GZ_FAN_MAX_PERCENT, 255);

	ret = lwmi_gz_fan_curve_set(priv, &curve);
	if (ret)
		return ret;

	return count;
}

static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point1_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 0);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point2_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 1);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point3_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 2);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point4_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 3);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point5_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 4);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point6_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 5);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point7_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 6);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point8_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 7);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point9_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 8);
static SENSOR_DEVICE_ATTR_2_RW(pwm1_auto_point10_pwm, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_PWM, 9);

static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point1_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 0);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point2_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 1);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point3_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 2);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point4_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 3);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point5_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 4);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point6_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 5);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point7_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 6);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point8_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 7);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point9_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 8);
static SENSOR_DEVICE_ATTR_2_RO(pwm1_auto_point10_temp, lwmi_gz_fan_curve,
			       LWMI_GZ_FAN_ATTR_CURVE_TEMP, 9);

#define LWMI_GZ_CURVE_ATTR_PTRS(_point) \
	&sensor_dev_attr_pwm1_auto_point##_point##_pwm.dev_attr.attr, \
	&sensor_dev_attr_pwm1_auto_point##_point##_temp.dev_attr.attr

static struct attribute *lwmi_gz_fan_attrs[] = {
	LWMI_GZ_CURVE_ATTR_PTRS(1),
	LWMI_GZ_CURVE_ATTR_PTRS(2),
	LWMI_GZ_CURVE_ATTR_PTRS(3),
	LWMI_GZ_CURVE_ATTR_PTRS(4),
	LWMI_GZ_CURVE_ATTR_PTRS(5),
	LWMI_GZ_CURVE_ATTR_PTRS(6),
	LWMI_GZ_CURVE_ATTR_PTRS(7),
	LWMI_GZ_CURVE_ATTR_PTRS(8),
	LWMI_GZ_CURVE_ATTR_PTRS(9),
	LWMI_GZ_CURVE_ATTR_PTRS(10),
	NULL,
};

static const struct attribute_group lwmi_gz_fan_group = {
	.attrs = lwmi_gz_fan_attrs,
};

static const struct attribute_group *lwmi_gz_fan_groups[] = {
	&lwmi_gz_fan_group,
	NULL,
};

static int lwmi_gz_fan_probe(struct wmi_device *wdev, const void *context)
{
	struct device *dev = &wdev->dev;
	struct lwmi_gz_fan_curve curve;
	struct lwmi_gz_fan_priv *priv;
	struct device *hwmon_dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->wdev = wdev;
	mutex_init(&priv->lock);
	dev_set_drvdata(dev, priv);

	if (lwmi_gz_fan_curve_get(priv, &curve))
		return -ENODEV;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, "lenovo_wmi_gamezone_fan",
							   priv, lwmi_gz_fan_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct wmi_device_id lwmi_gz_fan_id_table[] = {
	{ LENOVO_GAMEZONE_FAN_GUID, NULL },
	{}
};

static struct wmi_driver lwmi_gz_fan_driver = {
	.driver = {
		.name = "lenovo_wmi_gamezone_fan",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = lwmi_gz_fan_id_table,
	.probe = lwmi_gz_fan_probe,
	.no_singleton = true,
};
module_wmi_driver(lwmi_gz_fan_driver);

MODULE_DEVICE_TABLE(wmi, lwmi_gz_fan_id_table);
MODULE_AUTHOR("Antheas Kapenekakis <lkml@antheas.dev>");
MODULE_DESCRIPTION("Lenovo GameZone Fan WMI Driver");
MODULE_LICENSE("GPL");
