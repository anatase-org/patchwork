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
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define S2MM006_STATUS_FIRST	0x0e
#define S2MM006_STATUS_LAST	0x1c
#define S2MM006_REG_BC_STATUS	0x0e
#define S2MM006_REG_ATTACH	0x11
#define S2MM006_REG_VBUS	0x14
#define S2MM006_REG_SWITCH_STATUS	0x1c
#define S2MM006_REG_SWITCH_COMMAND	0x4e
#define S2MM006_CONSUMER_COMMAND	0x12
#define S2MM006_PROVIDER_COMMAND	0x09
#define S2MM006_MONITOR_MS		500

enum s2mm006_mode {
	S2MM006_MODE_NONE,
	S2MM006_MODE_CONSUMER,
	S2MM006_MODE_PROVIDER,
};

struct s2mm006 {
	struct i2c_client *client;
	struct dentry *debugfs;
	struct delayed_work monitor;
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

static void s2mm006_monitor(struct work_struct *work)
{
	struct s2mm006 *pdic = container_of(to_delayed_work(work),
					    struct s2mm006, monitor);
	struct device *dev = &pdic->client->dev;
	enum s2mm006_mode mode;
	u8 command, status_bit;
	int bc, attach, vbus, sw, ret;

	bc = s2mm006_read(pdic, S2MM006_REG_BC_STATUS);
	if (bc < 0)
		goto reschedule;

	if (!bc) {
		pdic->attempted_mode = S2MM006_MODE_NONE;
		goto reschedule;
	}

	attach = s2mm006_read(pdic, S2MM006_REG_ATTACH);
	vbus = s2mm006_read(pdic, S2MM006_REG_VBUS);
	sw = s2mm006_read(pdic, S2MM006_REG_SWITCH_STATUS);
	if (attach < 0 || vbus < 0 || sw < 0)
		goto reschedule;

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
		goto reschedule;
	}

	if (pdic->attempted_mode != mode)
		pdic->attempted_mode = S2MM006_MODE_NONE;
	if (sw & status_bit) {
		pdic->attempted_mode = mode;
		goto reschedule;
	}
	if (pdic->attempted_mode == mode)
		goto reschedule;

	ret = s2mm006_write(pdic, S2MM006_REG_SWITCH_COMMAND, command);
	if (ret) {
		dev_warn_ratelimited(dev, "failed to enable %s path: %d\n",
				     mode == S2MM006_MODE_CONSUMER ? "consumer" : "provider",
				     ret);
	} else {
		pdic->attempted_mode = mode;
		dev_info(dev, "requested %s path after attach\n",
			 mode == S2MM006_MODE_CONSUMER ? "consumer" : "provider");
	}

reschedule:
	schedule_delayed_work(&pdic->monitor,
			      msecs_to_jiffies(S2MM006_MONITOR_MS));
}

static int s2mm006_status_show(struct seq_file *seq, void *unused)
{
	struct device *dev = seq->private;
	struct s2mm006 *pdic = dev_get_drvdata(dev);
	int reg, value;

	for (reg = S2MM006_STATUS_FIRST; reg <= S2MM006_STATUS_LAST; reg++) {
		value = s2mm006_read(pdic, reg);
		if (value < 0)
			seq_printf(seq, "%02x: error %d\n", reg, value);
		else
			seq_printf(seq, "%02x: %02x\n", reg, value);
	}

	return 0;
}

static void s2mm006_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static int s2mm006_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s2mm006 *pdic;
	char *name;
	int status;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks combined I2C transfers\n");

	pdic = devm_kzalloc(dev, sizeof(*pdic), GFP_KERNEL);
	if (!pdic)
		return -ENOMEM;

	pdic->client = client;
	i2c_set_clientdata(client, pdic);
	INIT_DELAYED_WORK(&pdic->monitor, s2mm006_monitor);

	status = s2mm006_read(pdic, S2MM006_STATUS_LAST);
	if (status < 0)
		return dev_err_probe(dev, status, "unable to read PDIC status\n");

	name = devm_kasprintf(dev, GFP_KERNEL, "s2mm006-%s", dev_name(dev));
	if (!name)
		return -ENOMEM;

	pdic->debugfs = debugfs_create_dir(name, NULL);
	if (!IS_ERR_OR_NULL(pdic->debugfs)) {
		ret = devm_add_action_or_reset(dev, s2mm006_debugfs_remove,
					       pdic->debugfs);
		if (ret)
			return ret;

		debugfs_create_devm_seqfile(dev, "status", pdic->debugfs,
					    s2mm006_status_show);
	}
	dev_info(dev, "S2MM006 power-path monitor bound; switch status: 0x%02x\n",
		 status);
	schedule_delayed_work(&pdic->monitor, 0);

	return 0;
}

static void s2mm006_remove(struct i2c_client *client)
{
	struct s2mm006 *pdic = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&pdic->monitor);
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
	.remove = s2mm006_remove,
};
module_i2c_driver(s2mm006_driver);

MODULE_DESCRIPTION("Samsung S2MM006 USB Type-C power-path monitor");
MODULE_LICENSE("GPL");
