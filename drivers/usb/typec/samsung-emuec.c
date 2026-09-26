// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 *
 * Samsung EmuEC Type-C controller for the S2MM006 on Galaxy Book4 Edge.
 * The controller firmware owns the PD policy engine. Linux selects a sink
 * contract through its mailbox and reports the resulting Type-C state.
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/usb/pd.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/workqueue.h>

#define S2MM006_IRQ_FIRST		0x02
#define S2MM006_IRQ_LAST		0x07
#define S2MM006_REG_BC_STATUS		0x0e
#define S2MM006_REG_CC_STATUS		0x11
#define S2MM006_REG_PD_STATUS2		0x14
#define S2MM006_REG_SWITCH_STATUS	0x1c
#define S2MM006_REG_SWITCH_COMMAND	0x4e
#define S2MM006_REG_TX_STATUS		0x50
#define S2MM006_REG_ENTER_USB_STATUS	0x56
#define S2MM006_REG_TX_BUFFER		0x80
#define S2MM006_REG_ACTIVE_RDO		0x6a4
#define S2MM006_REG_SOURCE_CAPS		0x900

#define S2MM006_CONSUMER_COMMAND	0x12
#define S2MM006_PROVIDER_COMMAND	0x09
#define S2MM006_TX_REQUEST		BIT(6)
#define S2MM006_ENTER_USB_SUCCESS	(2 << 2)
#define S2MM006_MAX_CHARGE_MW		65000
#define S2MM006_MAX_CHARGE_MV		20000

enum samsung_emuec_mode {
	SAMSUNG_EMUEC_MODE_NONE,
	SAMSUNG_EMUEC_MODE_CONSUMER,
	SAMSUNG_EMUEC_MODE_PROVIDER,
};

struct samsung_emuec {
	struct i2c_client *client;
	struct mutex lock;
	struct delayed_work work;
	struct device_node *connector;
	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_power_delivery *pd;
	struct usb_power_delivery *partner_pd;
	struct usb_power_delivery_capabilities *source_caps;
	struct usb_role_switch *role_sw;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum samsung_emuec_mode attempted_mode;
	enum typec_role power_role;
	enum typec_data_role data_role;
	enum typec_orientation orientation;
	enum usb_role usb_role;
	enum usb_mode usb_mode;
	u32 voltage_uv;
	u32 current_ua;
	bool attached;
	bool usb_mux_active;
	bool pd_attempted;
	u8 caps_retries;
};

static int samsung_emuec_read(struct samsung_emuec *pdic, u16 reg,
			      void *buf, size_t len)
{
	u8 address[] = { reg >> 8, reg & 0xff };
	struct i2c_msg messages[] = {
		{ .addr = pdic->client->addr, .len = sizeof(address),
		  .buf = address },
		{ .addr = pdic->client->addr, .flags = I2C_M_RD,
		  .len = len, .buf = buf },
	};
	int ret;

	ret = i2c_transfer(pdic->client->adapter, messages, ARRAY_SIZE(messages));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(messages))
		return -EIO;

	return 0;
}

static int samsung_emuec_write(struct samsung_emuec *pdic, u16 reg,
			       const void *buf, size_t len)
{
	u8 data[2 + 8];
	int ret;

	if (len > sizeof(data) - 2)
		return -EINVAL;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;
	memcpy(data + 2, buf, len);

	ret = i2c_master_send(pdic->client, data, len + 2);
	if (ret < 0)
		return ret;
	if (ret != len + 2)
		return -EIO;

	return 0;
}

static int samsung_emuec_clear_irqs(struct samsung_emuec *pdic)
{
	u8 pending[S2MM006_IRQ_LAST - S2MM006_IRQ_FIRST + 1];
	int reg, ret;

	for (reg = S2MM006_IRQ_FIRST; reg <= S2MM006_IRQ_LAST; reg++) {
		ret = samsung_emuec_read(pdic, reg,
					&pending[reg - S2MM006_IRQ_FIRST],
					sizeof(pending[0]));
		if (ret)
			return ret;
	}

	/* The Windows driver acknowledges each interrupt byte by writing it back. */
	for (reg = S2MM006_IRQ_FIRST; reg <= S2MM006_IRQ_LAST; reg++) {
		ret = samsung_emuec_write(pdic, reg,
				     &pending[reg - S2MM006_IRQ_FIRST],
				     sizeof(pending[0]));
		if (ret)
			return ret;
	}

	return 0;
}

static int samsung_emuec_get_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      union power_supply_propval *val)
{
	struct samsung_emuec *pdic = power_supply_get_drvdata(psy);
	int ret = 0;

	guard(mutex)(&pdic->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pdic->attached && pdic->power_role == TYPEC_SINK;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pdic->voltage_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = pdic->current_ua;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property samsung_emuec_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static void samsung_emuec_unregister_partner(struct samsung_emuec *pdic)
{
	usb_power_delivery_unregister_capabilities(pdic->source_caps);
	pdic->source_caps = NULL;

	if (pdic->partner_pd) {
		typec_partner_set_usb_power_delivery(pdic->partner, NULL);
		usb_power_delivery_unregister(pdic->partner_pd);
		pdic->partner_pd = NULL;
	}

	if (pdic->partner) {
		typec_unregister_partner(pdic->partner);
		pdic->partner = NULL;
	}
}

static void samsung_emuec_detach(struct samsung_emuec *pdic)
{
	if (!pdic->attached && pdic->attempted_mode == SAMSUNG_EMUEC_MODE_NONE)
		return;

	if (pdic->attached) {
		usb_role_switch_set_role(pdic->role_sw, USB_ROLE_NONE);
		pdic->usb_role = USB_ROLE_NONE;
		typec_set_orientation(pdic->port, TYPEC_ORIENTATION_NONE);
		pdic->orientation = TYPEC_ORIENTATION_NONE;
		typec_set_mode(pdic->port, TYPEC_STATE_SAFE);
		pdic->usb_mux_active = false;
		typec_port_set_usb_mode(pdic->port, USB_MODE_NONE);
		typec_set_pwr_opmode(pdic->port, TYPEC_PWR_MODE_USB);
		samsung_emuec_unregister_partner(pdic);
	}

	pdic->attached = false;
	pdic->pd_attempted = false;
	pdic->usb_mode = USB_MODE_NONE;
	pdic->caps_retries = 0;
	pdic->attempted_mode = SAMSUNG_EMUEC_MODE_NONE;
	pdic->voltage_uv = 0;
	pdic->current_ua = 0;
	if (pdic->psy)
		power_supply_changed(pdic->psy);
}

static int samsung_emuec_read_source_caps(struct samsung_emuec *pdic,
					  u32 *pdos, int *count, u16 *revision)
{
	u8 buf[4 + PDO_MAX_OBJECTS * sizeof(u32)];
	u16 header;
	int i, ret;

	ret = samsung_emuec_read(pdic, S2MM006_REG_SOURCE_CAPS,
				buf, sizeof(buf));
	if (ret)
		return ret;
	
	header = get_unaligned_le16(buf);
	*count = pd_header_cnt(header);

	if (pd_header_type(header) != PD_DATA_SOURCE_CAP || !*count)
		return -ENODATA;

	if (pdo_type(get_unaligned_le32(buf + 4)) != PDO_TYPE_FIXED ||
	    pdo_fixed_voltage(get_unaligned_le32(buf + 4)) != 5000)
		return -ENODATA;

	*revision = pd_header_rev(header) == PD_REV30 ? 0x0300 : 0x0200;
	for (i = 0; i < *count; i++)
		pdos[i] = get_unaligned_le32(buf + 4 + i * sizeof(u32));

	return 0;
}

static int samsung_emuec_choose_pdo(const u32 *pdos, int count,
				    unsigned int *mv, unsigned int *ma)
{
	u32 best_power = 0;
	int best = 0, i;

	for (i = 0; i < count; i++) {
		u32 voltage, current_ma, power;

		if (pdo_type(pdos[i]) != PDO_TYPE_FIXED)
			continue;
		voltage = pdo_fixed_voltage(pdos[i]);
		if (!voltage || voltage > S2MM006_MAX_CHARGE_MV)
			continue;
		current_ma = min(pdo_max_current(pdos[i]),
				  S2MM006_MAX_CHARGE_MW * 1000U / voltage);
		current_ma = round_down(current_ma, 10);
		power = voltage * current_ma / 1000;
		if (power > best_power) {
			best_power = power;
			best = i + 1;
			*mv = voltage;
			*ma = current_ma;
		}
	}

	return best;
}

static int samsung_emuec_active_rdo(struct samsung_emuec *pdic, u32 *rdo)
{
	__le32 value;
	int ret;

	ret = samsung_emuec_read(pdic, S2MM006_REG_ACTIVE_RDO,
				&value, sizeof(value));
	if (ret)
		return ret;

	*rdo = le32_to_cpu(value);
	return 0;
}

static int samsung_emuec_request_pdo(struct samsung_emuec *pdic, u32 rdo)
{
	u8 request[8], status;
	u32 active;
	int ret, i;

	ret = samsung_emuec_read(pdic, S2MM006_REG_TX_STATUS,
				&status, sizeof(status));
	if (ret)
		return ret;
	if (status & S2MM006_TX_REQUEST)
		return -EBUSY;

	/* The EC mailbox expects a PD Request header and a reserved halfword. */
	put_unaligned_le16(0x1002, request);
	put_unaligned_le16(0, request + 2);
	put_unaligned_le32(rdo, request + 4);
	ret = samsung_emuec_write(pdic, S2MM006_REG_TX_BUFFER,
				 request, sizeof(request));
	if (ret)
		return ret;

	status |= S2MM006_TX_REQUEST;
	ret = samsung_emuec_write(pdic, S2MM006_REG_TX_STATUS,
				  &status, sizeof(status));
	if (ret)
		return ret;

	for (i = 0; i < 20; i++) {
		msleep(100);
		ret = samsung_emuec_active_rdo(pdic, &active);
		if (ret)
			return ret;
		if (rdo_index(active) == rdo_index(rdo) &&
		    rdo_op_current(active) == rdo_op_current(rdo))
			return 0;
	}

	return -ETIMEDOUT;
}

static void samsung_emuec_update_pd(struct samsung_emuec *pdic)
{
	struct usb_power_delivery_capabilities_desc caps = { .role = TYPEC_SOURCE };
	struct usb_power_delivery_desc desc;
	u32 pdos[PDO_MAX_OBJECTS] = {}, active, requested;
	unsigned int mv = 0, ma = 0;
	u16 revision;
	int count, selected, actual, ret;

	ret = samsung_emuec_read_source_caps(pdic, pdos, &count, &revision);
	if (ret) {
		if (ret == -ENODATA && pdic->caps_retries++ < 5)
			mod_delayed_work(system_dfl_wq, &pdic->work,
					 msecs_to_jiffies(200));
		return;
	}

	if (!pdic->partner_pd) {
		desc.revision = revision;
		desc.version = 0;
		pdic->partner_pd = typec_partner_usb_power_delivery_register(
							pdic->partner, &desc);
		if (!IS_ERR(pdic->partner_pd)) {
			memcpy(caps.pdo, pdos, sizeof(pdos));
			pdic->source_caps = usb_power_delivery_register_capabilities(
							pdic->partner_pd, &caps);
			if (IS_ERR(pdic->source_caps))
				pdic->source_caps = NULL;
			ret = typec_partner_set_usb_power_delivery(pdic->partner,
							pdic->partner_pd);
			if (ret) {
				usb_power_delivery_unregister_capabilities(
							pdic->source_caps);
				pdic->source_caps = NULL;
				usb_power_delivery_unregister(pdic->partner_pd);
				pdic->partner_pd = NULL;
			}
		} else {
			pdic->partner_pd = NULL;
		}
	}

	selected = samsung_emuec_choose_pdo(pdos, count, &mv, &ma);
	if (!selected)
		return;
	ret = samsung_emuec_active_rdo(pdic, &active);
	if (ret)
		return;

	requested = RDO_FIXED(selected, ma, ma, RDO_USB_COMM | RDO_NO_SUSPEND);
	actual = rdo_index(active);
	/* Firmware may already hold a stronger contract. Do not renegotiate it. */
	if (actual < 1 || actual > count ||
		pdo_type(pdos[actual - 1]) != PDO_TYPE_FIXED ||
		rdo_op_current(active) > pdo_max_current(pdos[actual - 1]) ||
		pdo_fixed_voltage(pdos[actual - 1]) *
		rdo_op_current(active) < mv * ma) {
		if (!pdic->pd_attempted) {
			pdic->pd_attempted = true;
			ret = samsung_emuec_request_pdo(pdic, requested);
			if (ret)
				dev_warn(&pdic->client->dev,
						"PD contract request failed: %d\n", ret);
			ret = samsung_emuec_active_rdo(pdic, &active);
			if (ret)
				return;
		}
	}

	actual = rdo_index(active);
	if (actual < 1 || actual > count ||
	    pdo_type(pdos[actual - 1]) != PDO_TYPE_FIXED)
		return;

	mv = pdo_fixed_voltage(pdos[actual - 1]);
	ma = min(rdo_op_current(active), pdo_max_current(pdos[actual - 1]));
	if (!mv || !ma)
		return;
	if (pdic->voltage_uv == mv * 1000 && pdic->current_ua == ma * 1000)
		return;
	pdic->voltage_uv = mv * 1000;
	pdic->current_ua = ma * 1000;
	typec_set_pwr_opmode(pdic->port, TYPEC_PWR_MODE_PD);
	power_supply_changed(pdic->psy);

	dev_info(&pdic->client->dev, "PD contract %u mV %u mA (PDO %d)\n",
		 mv, ma, actual);
}

static void samsung_emuec_sync(struct work_struct *work)
{
	struct samsung_emuec *pdic = container_of(to_delayed_work(work),
						    struct samsung_emuec, work);
	u8 command, status_bit, bc, cc, pd, sw, enter_usb;
	struct typec_partner_desc partner_desc = {
		.accessory = TYPEC_ACCESSORY_NONE,
		.usb_capability = 0,
	};
	enum samsung_emuec_mode mode;
	enum typec_orientation orientation;
	enum typec_role power_role;
	enum typec_data_role data_role;
	bool vbus_present, source_path_attached, consumer_path_active;
	int ret;

	guard(mutex)(&pdic->lock);

	ret = samsung_emuec_read(pdic, S2MM006_REG_BC_STATUS, &bc, sizeof(bc));
	if (ret)
		return;
	ret = samsung_emuec_read(pdic, S2MM006_REG_CC_STATUS, &cc, sizeof(cc));
	if (ret)
		return;
	ret = samsung_emuec_read(pdic, S2MM006_REG_PD_STATUS2, &pd, sizeof(pd));
	if (ret)
		return;
	ret = samsung_emuec_read(pdic, S2MM006_REG_SWITCH_STATUS, &sw, sizeof(sw));
	if (ret)
		return;
	ret = samsung_emuec_read(pdic, S2MM006_REG_ENTER_USB_STATUS,
				&enter_usb, sizeof(enter_usb));
	if (ret)
		return;

	orientation = (cc >> 4) == 1 ? TYPEC_ORIENTATION_NORMAL :
		      (cc >> 4) == 2 ? TYPEC_ORIENTATION_REVERSE :
					  TYPEC_ORIENTATION_NONE;
	power_role = pd & BIT(3) ? TYPEC_SOURCE : TYPEC_SINK;
	data_role = pd & BIT(2) ? TYPEC_HOST : TYPEC_DEVICE;
	vbus_present = bc & BIT(0);
	source_path_attached = bc & BIT(4);
	consumer_path_active = sw & BIT(1);

	/* A missing attach indication or CC orientation means a disconnect. */
	if (orientation == TYPEC_ORIENTATION_NONE ||
	    (!vbus_present && !source_path_attached)) {
		samsung_emuec_detach(pdic);
		return;
	}

	/* The BC and PD role bits can settle at different times. */
	if ((power_role == TYPEC_SOURCE && !source_path_attached) ||
	    (power_role == TYPEC_SINK && !vbus_present))
		return;

	mode = power_role == TYPEC_SOURCE ? SAMSUNG_EMUEC_MODE_PROVIDER :
					    SAMSUNG_EMUEC_MODE_CONSUMER;
	command = mode == SAMSUNG_EMUEC_MODE_PROVIDER ?
		  S2MM006_PROVIDER_COMMAND : S2MM006_CONSUMER_COMMAND;
	status_bit = mode == SAMSUNG_EMUEC_MODE_PROVIDER ? BIT(0) : BIT(1);

	if (pdic->attempted_mode != mode)
		pdic->attempted_mode = SAMSUNG_EMUEC_MODE_NONE;

	if (!(sw & status_bit) && pdic->attempted_mode != mode) {
		ret = samsung_emuec_write(pdic, S2MM006_REG_SWITCH_COMMAND,
					 &command, sizeof(command));
		if (ret)
			dev_warn(&pdic->client->dev,
				 "failed to enable %s path: %d\n",
				 mode == SAMSUNG_EMUEC_MODE_PROVIDER ?
				 "provider" : "consumer", ret);
		else {
			pdic->attempted_mode = mode;
			mod_delayed_work(system_dfl_wq, &pdic->work,
					 msecs_to_jiffies(150));
		}
	}

	if (!(sw & status_bit))
		return;

	if (pdic->attached && (pdic->power_role != power_role ||
			       pdic->data_role != data_role))
		samsung_emuec_detach(pdic);

	if (!pdic->attached) {
		partner_desc.usb_pd = power_role == TYPEC_SINK;

		pdic->partner = typec_register_partner(pdic->port, &partner_desc);
		if (IS_ERR(pdic->partner)) {
			dev_warn(&pdic->client->dev,
				 "failed to register Type-C partner: %ld\n",
				 PTR_ERR(pdic->partner));
			pdic->partner = NULL;
			return;
		}

		pdic->attached = true;
		pdic->power_role = power_role;
		pdic->data_role = data_role;
		typec_port_set_usb_mode(pdic->port, USB_MODE_NONE);
		typec_set_pwr_opmode(pdic->port, TYPEC_PWR_MODE_USB);
		pdic->voltage_uv = 0;
		pdic->current_ua = 0;
		power_supply_changed(pdic->psy);
	}

	if (pdic->orientation != orientation) {
		ret = typec_set_orientation(pdic->port, orientation);
		if (ret)
			dev_warn(&pdic->client->dev,
				 "failed to set connector orientation: %d\n", ret);
		else
			pdic->orientation = orientation;
	}

	if (!pdic->usb_mux_active) {
		ret = typec_set_mode(pdic->port, TYPEC_STATE_USB);
		if (ret)
			dev_warn(&pdic->client->dev,
				 "failed to enable USB mux: %d\n", ret);
		else
			pdic->usb_mux_active = true;
	}

	typec_set_pwr_role(pdic->port, power_role);
	typec_set_data_role(pdic->port, data_role);

	/* Windows names status 2 in these bits the Enter_USB success response. */
	if ((enter_usb & GENMASK(3, 2)) == S2MM006_ENTER_USB_SUCCESS) {
		if (pdic->usb_mode != USB_MODE_USB4) {
			/* A USB4 host router must own the USB4 mux transition. */
			typec_port_set_usb_mode(pdic->port, USB_MODE_USB4);
			pdic->usb_mode = USB_MODE_USB4;
		}
	} else if (pdic->usb_mode == USB_MODE_USB4) {
		typec_port_set_usb_mode(pdic->port, USB_MODE_NONE);
		pdic->usb_mode = USB_MODE_NONE;
	}

	if (pdic->usb_role != (data_role == TYPEC_HOST ? USB_ROLE_HOST :
							       USB_ROLE_DEVICE)) {
		enum usb_role usb_role = data_role == TYPEC_HOST ?
					USB_ROLE_HOST : USB_ROLE_DEVICE;

		ret = usb_role_switch_set_role(pdic->role_sw, usb_role);
		if (ret)
			dev_warn(&pdic->client->dev,
				 "failed to set USB role: %d\n", ret);
		else
			pdic->usb_role = usb_role;
	}

	if (power_role == TYPEC_SINK && consumer_path_active)
		samsung_emuec_update_pd(pdic);
}

static irqreturn_t samsung_emuec_irq_thread(int irq, void *data)
{
	struct samsung_emuec *pdic = data;
	int ret;

	scoped_guard(mutex, &pdic->lock)
		ret = samsung_emuec_clear_irqs(pdic);
	if (ret)
		dev_warn_ratelimited(&pdic->client->dev,
				     "failed to clear interrupt: %d\n", ret);

	mod_delayed_work(system_dfl_wq, &pdic->work, 0);
	return IRQ_HANDLED;
}

static int samsung_emuec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_config = {};
	struct usb_power_delivery_desc pd_desc = { .revision = 0x0300 };
	struct typec_capability typec_cap = {
		.type = TYPEC_PORT_DRP,
		.data = TYPEC_PORT_DRD,
		.revision = USB_TYPEC_REV_2_0,
		.pd_revision = 0x0300,
		.orientation_aware = true,
		.usb_capability = USB_CAPABILITY_USB2 | USB_CAPABILITY_USB3 |
				  USB_CAPABILITY_USB4,
		.no_mode_control = true,
	};
	struct samsung_emuec *pdic;
	u8 status;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks combined I2C transfers\n");

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "missing interrupt\n");

	pdic = devm_kzalloc(dev, sizeof(*pdic), GFP_KERNEL);
	if (!pdic)
		return -ENOMEM;

	pdic->client = client;
	mutex_init(&pdic->lock);
	INIT_DELAYED_WORK(&pdic->work, samsung_emuec_sync);
	i2c_set_clientdata(client, pdic);

	ret = samsung_emuec_read(pdic, S2MM006_REG_SWITCH_STATUS,
				&status, sizeof(status));
	if (ret)
		return dev_err_probe(dev, ret, "unable to read PDIC status\n");

	pdic->connector = of_get_child_by_name(dev->of_node, "connector");
	if (!pdic->connector)
		return dev_err_probe(dev, -EINVAL, "missing Type-C connector\n");

	typec_cap.fwnode = of_fwnode_handle(pdic->connector);
	typec_cap.driver_data = pdic;

	pdic->role_sw = fwnode_usb_role_switch_get(typec_cap.fwnode);
	if (IS_ERR(pdic->role_sw)) {
		ret = PTR_ERR(pdic->role_sw);
		goto err_node;
	}
	if (!pdic->role_sw) {
		ret = -ENODEV;
		goto err_node;
	}

	pdic->port = typec_register_port(dev, &typec_cap);
	if (IS_ERR(pdic->port)) {
		ret = PTR_ERR(pdic->port);
		goto err_role;
	}

	pdic->pd = usb_power_delivery_register(dev, &pd_desc);
	if (IS_ERR(pdic->pd)) {
		ret = PTR_ERR(pdic->pd);
		goto err_port;
	}

	ret = typec_port_set_usb_power_delivery(pdic->port, pdic->pd);
	if (ret)
		goto err_pd;

	pdic->psy_desc.name = devm_kasprintf(dev, GFP_KERNEL, "%s-usb",
					       dev_name(dev));
	if (!pdic->psy_desc.name) {
		ret = -ENOMEM;
		goto err_pd;
	}

	pdic->psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
	pdic->psy_desc.properties = samsung_emuec_properties;
	pdic->psy_desc.num_properties = ARRAY_SIZE(samsung_emuec_properties);
	pdic->psy_desc.get_property = samsung_emuec_get_property;
	psy_config.drv_data = pdic;
	psy_config.fwnode = of_fwnode_handle(pdic->connector);

	pdic->psy = devm_power_supply_register(dev, &pdic->psy_desc, &psy_config);
	if (IS_ERR(pdic->psy)) {
		ret = PTR_ERR(pdic->psy);
		goto err_pd;
	}

	ret = samsung_emuec_clear_irqs(pdic);
	if (ret)
		goto err_pd;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					samsung_emuec_irq_thread, IRQF_ONESHOT,
					dev_name(dev), pdic);
	if (ret)
		goto err_pd;

	mod_delayed_work(system_dfl_wq, &pdic->work, 0);
	return 0;

err_pd:
	typec_port_set_usb_power_delivery(pdic->port, NULL);
	usb_power_delivery_unregister(pdic->pd);
err_port:
	typec_unregister_port(pdic->port);
err_role:
	usb_role_switch_put(pdic->role_sw);
err_node:
	of_node_put(pdic->connector);
	return ret;
}

static void samsung_emuec_remove(struct i2c_client *client)
{
	struct samsung_emuec *pdic = i2c_get_clientdata(client);

	devm_free_irq(&client->dev, client->irq, pdic);
	cancel_delayed_work_sync(&pdic->work);

	scoped_guard(mutex, &pdic->lock)
		samsung_emuec_detach(pdic);

	typec_port_set_usb_power_delivery(pdic->port, NULL);
	usb_power_delivery_unregister(pdic->pd);
	typec_unregister_port(pdic->port);
	usb_role_switch_put(pdic->role_sw);
	of_node_put(pdic->connector);
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
	.remove = samsung_emuec_remove,
};
module_i2c_driver(samsung_emuec_driver);

MODULE_DESCRIPTION("Samsung EmuEC USB Type-C and Power Delivery driver");
MODULE_LICENSE("GPL");
