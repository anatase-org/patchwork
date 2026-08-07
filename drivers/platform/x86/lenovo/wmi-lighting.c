// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Lenovo Lighting WMI interface driver.
 *
 * Copyright (C) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wmi.h>

#define LENOVO_LIGHTING_GUID		"8C5B9127-ECD4-4657-980F-851019F99CA5"

#define LWMI_LIGHTING_METHOD_GET	1
#define LWMI_LIGHTING_METHOD_SET	2

#define LWMI_LIGHTING_ID_COMBINED	0x03
#define LWMI_LIGHTING_ID_POWER		0x04
#define LWMI_LIGHTING_ID_STANDBY		0x24

#define LWMI_LIGHTING_STATE_OFF		0x01
#define LWMI_LIGHTING_STATE_POWER_ON	0x02
#define LWMI_LIGHTING_STATE_STANDBY_ON	0x03

enum lwmi_lighting_type {
	LWMI_LIGHTING_COMBINED,
	LWMI_LIGHTING_POWER,
	LWMI_LIGHTING_STANDBY,
};

struct lwmi_lighting_priv;

struct lwmi_lighting_led {
	struct led_classdev cdev;
	struct lwmi_lighting_priv *priv;
	enum lwmi_lighting_type type;
	u8 id;
};

struct lwmi_lighting_priv {
	struct wmi_device *wdev;
	struct mutex lock; /* WMI method serialization */
	struct lwmi_lighting_led leds[2];
};

static int lwmi_lighting_get(struct lwmi_lighting_priv *priv, u8 id,
			     enum lwmi_lighting_type type,
			     enum led_brightness *brightness)
{
	struct wmi_buffer output = {};
	struct wmi_buffer input = {
		.length = sizeof(id),
		.data = &id,
	};
	u8 *status;
	int ret;

	ret = wmidev_invoke_method(priv->wdev, 0, LWMI_LIGHTING_METHOD_GET,
				   &input, &output);
	if (ret)
		return ret;
	if (output.length != 2) {
		ret = -ENODATA;
		goto out_free;
	}

	status = output.data;
	switch (type) {
	case LWMI_LIGHTING_COMBINED:
		if (status[0] > 1 || status[1] != 0) {
			ret = -ENODATA;
			break;
		}

		*brightness = status[0] ? LED_ON : LED_OFF;
		break;
	case LWMI_LIGHTING_POWER:
		if (status[0] != 1 ||
		    (status[1] != LWMI_LIGHTING_STATE_OFF &&
		     status[1] != LWMI_LIGHTING_STATE_POWER_ON)) {
			ret = -ENODATA;
			break;
		}

		*brightness = status[1] == LWMI_LIGHTING_STATE_POWER_ON ?
			      LED_ON : LED_OFF;
		break;
	case LWMI_LIGHTING_STANDBY:
		if (status[0] != 1 ||
		    (status[1] != LWMI_LIGHTING_STATE_OFF &&
		     status[1] != LWMI_LIGHTING_STATE_STANDBY_ON)) {
			ret = -ENODATA;
			break;
		}

		*brightness = status[1] == LWMI_LIGHTING_STATE_STANDBY_ON ?
			      LED_ON : LED_OFF;
		break;
	}

out_free:
	kfree(output.data);
	return ret;
}

static int lwmi_lighting_set(struct lwmi_lighting_led *led,
			     enum led_brightness brightness)
{
	u8 args[3] = { led->id };
	struct wmi_buffer input = {
		.length = sizeof(args),
		.data = args,
	};

	switch (led->type) {
	case LWMI_LIGHTING_COMBINED:
		args[1] = brightness != LED_OFF;
		break;
	case LWMI_LIGHTING_POWER:
		args[2] = brightness == LED_OFF ? LWMI_LIGHTING_STATE_OFF :
			  LWMI_LIGHTING_STATE_POWER_ON;
		break;
	case LWMI_LIGHTING_STANDBY:
		args[2] = brightness == LED_OFF ? LWMI_LIGHTING_STATE_OFF :
			  LWMI_LIGHTING_STATE_STANDBY_ON;
		break;
	}

	return wmidev_invoke_method(led->priv->wdev, 0,
				    LWMI_LIGHTING_METHOD_SET, &input, NULL);
}

static enum led_brightness
lwmi_lighting_brightness_get(struct led_classdev *cdev)
{
	struct lwmi_lighting_led *led = container_of(cdev,
						      struct lwmi_lighting_led,
						      cdev);
	enum led_brightness brightness;
	int ret;

	mutex_lock(&led->priv->lock);
	ret = lwmi_lighting_get(led->priv, led->id, led->type, &brightness);
	mutex_unlock(&led->priv->lock);
	if (ret) {
		dev_warn_ratelimited(&led->priv->wdev->dev,
				     "Failed to get %s LED brightness: %d\n",
				     cdev->name, ret);
		return cdev->brightness;
	}

	return brightness;
}

static int lwmi_lighting_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct lwmi_lighting_led *led = container_of(cdev,
						      struct lwmi_lighting_led,
						      cdev);
	int ret;

	mutex_lock(&led->priv->lock);
	ret = lwmi_lighting_set(led, brightness);
	mutex_unlock(&led->priv->lock);

	return ret;
}

static int lwmi_lighting_register_led(struct lwmi_lighting_priv *priv,
				      unsigned int index, const char *name,
				      u8 id, enum lwmi_lighting_type type,
				      enum led_brightness brightness)
{
	struct lwmi_lighting_led *led = &priv->leds[index];

	led->priv = priv;
	led->id = id;
	led->type = type;
	led->cdev.name = name;
	led->cdev.max_brightness = LED_ON;
	led->cdev.brightness = brightness;
	led->cdev.brightness_get = lwmi_lighting_brightness_get;
	led->cdev.brightness_set_blocking = lwmi_lighting_brightness_set;
	led->cdev.flags = LED_RETAIN_AT_SHUTDOWN;

	return devm_led_classdev_register(&priv->wdev->dev, &led->cdev);
}

static int lwmi_lighting_probe(struct wmi_device *wdev, const void *context)
{
	enum led_brightness combined_brightness;
	enum led_brightness standby_brightness;
	enum led_brightness power_brightness;
	struct lwmi_lighting_priv *priv;
	bool power_supported;
	int ret;

	priv = devm_kzalloc(&wdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->wdev = wdev;
	mutex_init(&priv->lock);
	dev_set_drvdata(&wdev->dev, priv);

	/*
	 * Query power first to populate the firmware's shared return buffer.
	 * Unsupported IDs leave that buffer unchanged, so the following combined
	 * query can then be distinguished from a split-only implementation.
	 */
	ret = lwmi_lighting_get(priv, LWMI_LIGHTING_ID_POWER,
				LWMI_LIGHTING_POWER, &power_brightness);
	power_supported = !ret;

	ret = lwmi_lighting_get(priv, LWMI_LIGHTING_ID_COMBINED,
				LWMI_LIGHTING_COMBINED, &combined_brightness);
	if (!ret) {
		dev_dbg(&wdev->dev, "Using combined power LED control\n");
		return lwmi_lighting_register_led(priv, 0, "platform::power",
						  LWMI_LIGHTING_ID_COMBINED,
						  LWMI_LIGHTING_COMBINED,
						  combined_brightness);
	}

	if (!power_supported)
		return -ENODEV;

	ret = lwmi_lighting_get(priv, LWMI_LIGHTING_ID_STANDBY,
				LWMI_LIGHTING_STANDBY, &standby_brightness);
	if (ret)
		return -ENODEV;

	ret = lwmi_lighting_register_led(priv, 0, "platform::power",
					 LWMI_LIGHTING_ID_POWER,
					 LWMI_LIGHTING_POWER, power_brightness);
	if (ret)
		return ret;

	dev_dbg(&wdev->dev, "Using separate power and standby LED controls\n");
	return lwmi_lighting_register_led(priv, 1, "platform::standby",
					  LWMI_LIGHTING_ID_STANDBY,
					  LWMI_LIGHTING_STANDBY,
					  standby_brightness);
}

static const struct wmi_device_id lwmi_lighting_id_table[] = {
	{ LENOVO_LIGHTING_GUID, NULL },
	{}
};

static struct wmi_driver lwmi_lighting_driver = {
	.driver = {
		.name = "lenovo_wmi_lighting",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = lwmi_lighting_id_table,
	.probe = lwmi_lighting_probe,
	.no_singleton = true,
};
module_wmi_driver(lwmi_lighting_driver);

MODULE_DEVICE_TABLE(wmi, lwmi_lighting_id_table);
MODULE_AUTHOR("Antheas Kapenekakis <lkml@antheas.dev>");
MODULE_DESCRIPTION("Lenovo Lighting WMI Driver");
MODULE_LICENSE("GPL");
