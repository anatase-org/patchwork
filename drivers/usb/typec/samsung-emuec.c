// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 *
 * Samsung EmuEC USB Type-C power-path driver for S2MM006 controllers.
 *
 * The Windows EmuEC driver addresses registers as two big-endian bytes and
 * reads the range below when handling port status. The consumer switch must
 * be requested again after a live charger attach, even if the command
 * register still contains the previous request. A USB host also needs the
 * provider switch requested after a data-device attach. Power-delivery
 * contract selection and USB data roles are not handled here yet.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>

#define S2MM006_IRQ_FIRST	0x02
#define S2MM006_IRQ_LAST	0x07
#define S2MM006_REG_BC_STATUS	0x0e
#define S2MM006_REG_ATTACH	0x11
#define S2MM006_REG_VBUS	0x14
#define S2MM006_REG_SWITCH_STATUS	0x1c
#define S2MM006_REG_SWITCH_COMMAND	0x4e
#define S2MM006_CONSUMER_COMMAND	0x12
#define S2MM006_PROVIDER_COMMAND	0x09

enum samsung_emuec_mode {
	SAMSUNG_EMUEC_MODE_NONE,
	SAMSUNG_EMUEC_MODE_CONSUMER,
	SAMSUNG_EMUEC_MODE_PROVIDER,
};

struct samsung_emuec {
	struct i2c_client *client;
	enum samsung_emuec_mode attempted_mode;
};

static int samsung_emuec_read(struct samsung_emuec *pdic, u16 reg)
{
	u8 address[] = { reg >> 8, reg & 0xff };
	u8 value;
	struct i2c_msg messages[] = {
		{
			.addr = pdic->client->addr,
			.len = sizeof(address),
			.buf = address,
		},
		{
			.addr = pdic->client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(value),
			.buf = &value,
		},
	};
	int ret;

	ret = i2c_transfer(pdic->client->adapter, messages, ARRAY_SIZE(messages));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(messages))
		return -EIO;

	return value;
}

static int samsung_emuec_write(struct samsung_emuec *pdic, u16 reg, u8 value)
{
	u8 data[] = { reg >> 8, reg & 0xff, value };
	int ret;

	ret = i2c_master_send(pdic->client, data, sizeof(data));
	if (ret < 0)
		return ret;
	if (ret != sizeof(data))
		return -EIO;

	return 0;
}

static int samsung_emuec_clear_irqs(struct samsung_emuec *pdic)
{
	u8 pending[S2MM006_IRQ_LAST - S2MM006_IRQ_FIRST + 1];
	int reg, ret;

	for (reg = S2MM006_IRQ_FIRST; reg <= S2MM006_IRQ_LAST; reg++) {
		ret = samsung_emuec_read(pdic, reg);
		if (ret < 0)
			return ret;
		pending[reg - S2MM006_IRQ_FIRST] = ret;
	}

	/* The firmware driver acknowledges by writing each value back. */
	for (reg = S2MM006_IRQ_FIRST; reg <= S2MM006_IRQ_LAST; reg++) {
		ret = samsung_emuec_write(pdic, reg,
				    pending[reg - S2MM006_IRQ_FIRST]);
		if (ret)
			return ret;
	}

	return 0;
}

static void samsung_emuec_check_status(struct samsung_emuec *pdic)
{
	struct device *dev = &pdic->client->dev;
	enum samsung_emuec_mode mode;
	u8 command, status_bit;
	int bc, attach, vbus, sw, ret;

	bc = samsung_emuec_read(pdic, S2MM006_REG_BC_STATUS);
	if (bc < 0)
		return;

	if (!bc) {
		pdic->attempted_mode = SAMSUNG_EMUEC_MODE_NONE;
		return;
	}

	attach = samsung_emuec_read(pdic, S2MM006_REG_ATTACH);
	vbus = samsung_emuec_read(pdic, S2MM006_REG_VBUS);
	sw = samsung_emuec_read(pdic, S2MM006_REG_SWITCH_STATUS);
	if (attach < 0 || vbus < 0 || sw < 0)
		return;

	/* Windows EmuEC treats bit 0 of BC_STATUS as VBUS detected. */
	if ((bc & BIT(0)) && attach == 0x1b && vbus == 0x01 &&
	    !(sw & BIT(0))) {
		mode = SAMSUNG_EMUEC_MODE_CONSUMER;
		command = S2MM006_CONSUMER_COMMAND;
		status_bit = BIT(1);
	} else if (!(bc & BIT(0)) && (bc & BIT(4)) &&
		   attach == 0x2e && vbus == 0x1c && !(sw & BIT(1))) {
		/* This signature was observed with a USB-C data device. */
		mode = SAMSUNG_EMUEC_MODE_PROVIDER;
		command = S2MM006_PROVIDER_COMMAND;
		status_bit = BIT(0);
	} else {
		return;
	}

	if (pdic->attempted_mode != mode)
		pdic->attempted_mode = SAMSUNG_EMUEC_MODE_NONE;
	if (sw & status_bit) {
		pdic->attempted_mode = mode;
		return;
	}
	if (pdic->attempted_mode == mode)
		return;

	ret = samsung_emuec_write(pdic, S2MM006_REG_SWITCH_COMMAND, command);
	if (ret) {
		dev_warn(dev, "failed to enable %s path: %d\n",
			 mode == SAMSUNG_EMUEC_MODE_CONSUMER ? "consumer" : "provider",
			 ret);
	} else {
		pdic->attempted_mode = mode;
		dev_info(dev, "requested %s path after attach\n",
			 mode == SAMSUNG_EMUEC_MODE_CONSUMER ? "consumer" : "provider");
	}
}

static irqreturn_t samsung_emuec_irq_thread(int irq, void *data)
{
	struct samsung_emuec *pdic = data;
	int ret;

	ret = samsung_emuec_clear_irqs(pdic);
	samsung_emuec_check_status(pdic);
	if (ret)
		dev_warn_ratelimited(&pdic->client->dev,
				     "failed to clear interrupt: %d\n", ret);

	return IRQ_HANDLED;
}

static int samsung_emuec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct samsung_emuec *pdic;
	int status;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks combined I2C transfers\n");

	pdic = devm_kzalloc(dev, sizeof(*pdic), GFP_KERNEL);
	if (!pdic)
		return -ENOMEM;

	pdic->client = client;

	status = samsung_emuec_read(pdic, S2MM006_REG_SWITCH_STATUS);
	if (status < 0)
		return dev_err_probe(dev, status, "unable to read PDIC status\n");

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "missing interrupt\n");

	ret = samsung_emuec_clear_irqs(pdic);
	if (ret)
		return dev_err_probe(dev, ret, "unable to clear pending interrupts\n");

	/* A later event holds the level-low line until the IRQ is requested. */
	samsung_emuec_check_status(pdic);

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					samsung_emuec_irq_thread, IRQF_ONESHOT,
					dev_name(dev), pdic);
	if (ret)
		return dev_err_probe(dev, ret, "unable to request interrupt\n");

	dev_info(dev, "EmuEC Type-C port ready; switch status: 0x%02x\n",
		 status);

	return 0;
}

static const struct of_device_id samsung_emuec_of_match[] = {
	{ .compatible = "samsung,s2mm006" },
	{ }
};
MODULE_DEVICE_TABLE(of, samsung_emuec_of_match);

static struct i2c_driver samsung_emuec_driver = {
	.driver = {
		.name = "samsung-emuec",
		.of_match_table = samsung_emuec_of_match,
	},
	.probe = samsung_emuec_probe,
};
module_i2c_driver(samsung_emuec_driver);

MODULE_DESCRIPTION("Samsung EmuEC USB Type-C power-path driver");
MODULE_LICENSE("GPL");
