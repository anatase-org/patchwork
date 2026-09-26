// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 *
 * Samsung S2MM006 USB Type-C controller on the Galaxy Book4 Edge NP750XQB.
 *
 * The Windows EMEC driver addresses registers as two big-endian bytes and
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

enum s2mm006_mode {
	S2MM006_MODE_NONE,
	S2MM006_MODE_CONSUMER,
	S2MM006_MODE_PROVIDER,
};

struct s2mm006 {
	struct i2c_client *client;
	enum s2mm006_mode attempted_mode;
};

static int s2mm006_read(struct s2mm006 *pdic, u16 reg)
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

static int s2mm006_write(struct s2mm006 *pdic, u16 reg, u8 value)
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

static int s2mm006_clear_irqs(struct s2mm006 *pdic)
{
	u8 pending[S2MM006_IRQ_LAST - S2MM006_IRQ_FIRST + 1];
	int reg, ret;

	for (reg = S2MM006_IRQ_FIRST; reg <= S2MM006_IRQ_LAST; reg++) {
		ret = s2mm006_read(pdic, reg);
		if (ret < 0)
			return ret;
		pending[reg - S2MM006_IRQ_FIRST] = ret;
	}

	/* The firmware driver acknowledges by writing each value back. */
	for (reg = S2MM006_IRQ_FIRST; reg <= S2MM006_IRQ_LAST; reg++) {
		ret = s2mm006_write(pdic, reg,
				    pending[reg - S2MM006_IRQ_FIRST]);
		if (ret)
			return ret;
	}

	return 0;
}

static void s2mm006_check_status(struct s2mm006 *pdic)
{
	struct device *dev = &pdic->client->dev;
	enum s2mm006_mode mode;
	u8 command, status_bit;
	int bc, attach, vbus, sw, ret;

	bc = s2mm006_read(pdic, S2MM006_REG_BC_STATUS);
	if (bc < 0)
		return;

	if (!bc) {
		pdic->attempted_mode = S2MM006_MODE_NONE;
		return;
	}

	attach = s2mm006_read(pdic, S2MM006_REG_ATTACH);
	vbus = s2mm006_read(pdic, S2MM006_REG_VBUS);
	sw = s2mm006_read(pdic, S2MM006_REG_SWITCH_STATUS);
	if (attach < 0 || vbus < 0 || sw < 0)
		return;

	/* Windows EMEC treats bit 0 of BC_STATUS as VBUS detected. */
	if ((bc & BIT(0)) && attach == 0x1b && vbus == 0x01 &&
	    !(sw & BIT(0))) {
		mode = S2MM006_MODE_CONSUMER;
		command = S2MM006_CONSUMER_COMMAND;
		status_bit = BIT(1);
	} else if (!(bc & BIT(0)) && (bc & BIT(4)) &&
		   attach == 0x2e && vbus == 0x1c && !(sw & BIT(1))) {
		/* This signature was observed with a USB-C data device. */
		mode = S2MM006_MODE_PROVIDER;
		command = S2MM006_PROVIDER_COMMAND;
		status_bit = BIT(0);
	} else {
		return;
	}

	if (pdic->attempted_mode != mode)
		pdic->attempted_mode = S2MM006_MODE_NONE;
	if (sw & status_bit) {
		pdic->attempted_mode = mode;
		return;
	}
	if (pdic->attempted_mode == mode)
		return;

	ret = s2mm006_write(pdic, S2MM006_REG_SWITCH_COMMAND, command);
	if (ret) {
		dev_warn(dev, "failed to enable %s path: %d\n",
			 mode == S2MM006_MODE_CONSUMER ? "consumer" : "provider",
			 ret);
	} else {
		pdic->attempted_mode = mode;
		dev_info(dev, "requested %s path after attach\n",
			 mode == S2MM006_MODE_CONSUMER ? "consumer" : "provider");
	}
}

static irqreturn_t s2mm006_irq_thread(int irq, void *data)
{
	struct s2mm006 *pdic = data;
	int ret;

	ret = s2mm006_clear_irqs(pdic);
	s2mm006_check_status(pdic);
	if (ret)
		dev_warn_ratelimited(&pdic->client->dev,
				     "failed to clear interrupt: %d\n", ret);

	return IRQ_HANDLED;
}

static int s2mm006_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s2mm006 *pdic;
	int status;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks combined I2C transfers\n");

	pdic = devm_kzalloc(dev, sizeof(*pdic), GFP_KERNEL);
	if (!pdic)
		return -ENOMEM;

	pdic->client = client;

	status = s2mm006_read(pdic, S2MM006_REG_SWITCH_STATUS);
	if (status < 0)
		return dev_err_probe(dev, status, "unable to read PDIC status\n");

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "missing interrupt\n");

	ret = s2mm006_clear_irqs(pdic);
	if (ret)
		return dev_err_probe(dev, ret, "unable to clear pending interrupts\n");

	/* A later event holds the level-low line until the IRQ is requested. */
	s2mm006_check_status(pdic);

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					s2mm006_irq_thread, IRQF_ONESHOT,
					dev_name(dev), pdic);
	if (ret)
		return dev_err_probe(dev, ret, "unable to request interrupt\n");

	dev_info(dev, "S2MM006 controller ready; switch status: 0x%02x\n",
		 status);

	return 0;
}

static const struct of_device_id s2mm006_of_match[] = {
	{ .compatible = "samsung,s2mm006" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mm006_of_match);

static struct i2c_driver s2mm006_driver = {
	.driver = {
		.name = "s2mm006",
		.of_match_table = s2mm006_of_match,
	},
	.probe = s2mm006_probe,
};
module_i2c_driver(s2mm006_driver);

MODULE_DESCRIPTION("Samsung S2MM006 USB Type-C power-path driver");
MODULE_LICENSE("GPL");
