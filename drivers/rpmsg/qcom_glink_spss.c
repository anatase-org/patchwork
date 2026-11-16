// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm SPSS GLINK transport
 * Copyright (c) 2018 The Linux Foundation. All rights reserved.
 * Copyright (c) 2026 Antheas Kapenekakis <lkml@antheas.dev>
 *
 * The SPSS expects the AP to allocate the GLINK descriptor and FIFOs as one
 * SMEM item, then publish that item's physical address and size in the SPSS
 * handoff registers. This differs from the three-item GLINK SMEM transport.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/qcom/smem.h>

#include "qcom_glink_native.h"

#define SPSS_GLINK_DESC_ITEM	478
#define SPSS_GLINK_FIFO_SIZE	SZ_2K
#define SPSS_GLINK_FIFO_RESERVE	16

struct spss_glink_config {
	__le32 tx_tail;
	__le32 tx_head;
	__le32 tx_fifo_size;
	__le32 rx_tail;
	__le32 rx_head;
	__le32 rx_fifo_size;
};

struct spss_glink;

struct spss_glink_pipe {
	struct qcom_glink_pipe native;
	struct spss_glink *edge;
	__le32 *head;
	__le32 *tail;
	u8 *fifo;
};

struct spss_glink {
	struct device *dev;
	struct device *glink_dev;
	struct qcom_glink *glink;
	struct spss_glink_pipe rx;
	struct spss_glink_pipe tx;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	void __iomem *handoff_addr;
	void __iomem *handoff_size;
	int irq;
};

static void spss_glink_dev_release(struct device *dev)
{
	kfree(dev);
}

static struct spss_glink_pipe *to_spss_pipe(struct qcom_glink_pipe *pipe)
{
	return container_of(pipe, struct spss_glink_pipe, native);
}

static size_t spss_rx_avail(struct qcom_glink_pipe *native)
{
	struct spss_glink_pipe *pipe = to_spss_pipe(native);
	u32 head = le32_to_cpu(READ_ONCE(*pipe->head));
	u32 tail = le32_to_cpu(READ_ONCE(*pipe->tail));

	if (head >= native->length || tail >= native->length)
		return 0;

	return (head + native->length - tail) % native->length;
}

static void spss_rx_peek(struct qcom_glink_pipe *native, void *data,
			 unsigned int offset, size_t count)
{
	struct spss_glink_pipe *pipe = to_spss_pipe(native);
	u32 tail = le32_to_cpu(READ_ONCE(*pipe->tail));
	size_t first;

	/* Pair the head read in spss_rx_avail() with the peer's FIFO writes. */
	rmb();
	tail = (tail + offset) % native->length;
	first = min(count, native->length - tail);
	memcpy(data, pipe->fifo + tail, first);
	memcpy((u8 *)data + first, pipe->fifo, count - first);
}

static void spss_rx_advance(struct qcom_glink_pipe *native, size_t count)
{
	struct spss_glink_pipe *pipe = to_spss_pipe(native);
	u32 tail = le32_to_cpu(READ_ONCE(*pipe->tail));

	tail = (tail + count) % native->length;
	WRITE_ONCE(*pipe->tail, cpu_to_le32(tail));
}

static size_t spss_tx_avail(struct qcom_glink_pipe *native)
{
	struct spss_glink_pipe *pipe = to_spss_pipe(native);
	u32 head = le32_to_cpu(READ_ONCE(*pipe->head));
	u32 tail = le32_to_cpu(READ_ONCE(*pipe->tail));
	size_t avail;

	if (head >= native->length || tail >= native->length)
		return 0;

	if (tail <= head)
		avail = native->length - head + tail;
	else
		avail = tail - head;

	return avail > SPSS_GLINK_FIFO_RESERVE ?
		avail - SPSS_GLINK_FIFO_RESERVE : 0;
}

static u32 spss_tx_copy(struct spss_glink_pipe *pipe, u32 head,
			const void *data, size_t count)
{
	size_t first = min(count, pipe->native.length - head);

	if (first)
		memcpy(pipe->fifo + head, data, first);
	if (count > first)
		memcpy(pipe->fifo, (const u8 *)data + first, count - first);
	return (head + count) % pipe->native.length;
}

static void spss_tx_write(struct qcom_glink_pipe *native, const void *hdr,
			  size_t hlen, const void *data, size_t dlen)
{
	struct spss_glink_pipe *pipe = to_spss_pipe(native);
	u32 head = le32_to_cpu(READ_ONCE(*pipe->head));

	head = spss_tx_copy(pipe, head, hdr, hlen);
	head = spss_tx_copy(pipe, head, data, dlen);
	head = ALIGN(head, 8) % native->length;
	/* Publish the FIFO contents before making the new head visible. */
	wmb();
	WRITE_ONCE(*pipe->head, cpu_to_le32(head));
}

static void spss_tx_kick(struct qcom_glink_pipe *native)
{
	struct spss_glink *edge = to_spss_pipe(native)->edge;

	mbox_send_message(edge->mbox_chan, NULL);
	mbox_client_txdone(edge->mbox_chan, 0);
}

static irqreturn_t spss_glink_irq(int irq, void *data)
{
	struct spss_glink *edge = data;

	qcom_glink_native_rx(edge->glink);
	return IRQ_HANDLED;
}

static int spss_glink_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spss_glink_config *cfg;
	struct spss_glink *edge;
	phys_addr_t phys;
	size_t size;
	u32 host;
	int ret;

	ret = of_property_read_u32(dev->of_node, "qcom,remote-pid", &host);
	if (ret)
		return ret;

	edge = devm_kzalloc(dev, sizeof(*edge), GFP_KERNEL);
	if (!edge)
		return -ENOMEM;
	edge->dev = dev;

	edge->handoff_addr = devm_platform_ioremap_resource_byname(pdev,
							      "handoff-address");
	if (IS_ERR(edge->handoff_addr))
		return PTR_ERR(edge->handoff_addr);
	edge->handoff_size = devm_platform_ioremap_resource_byname(pdev,
							      "handoff-size");
	if (IS_ERR(edge->handoff_size))
		return PTR_ERR(edge->handoff_size);

	edge->irq = platform_get_irq(pdev, 0);
	if (edge->irq < 0)
		return edge->irq;
	ret = devm_request_irq(dev, edge->irq, spss_glink_irq,
			       IRQF_NO_AUTOEN | IRQF_NO_SUSPEND, dev_name(dev), edge);
	if (ret)
		return ret;

	edge->mbox_client.dev = dev;
	edge->mbox_client.knows_txdone = true;
	edge->mbox_chan = mbox_request_channel(&edge->mbox_client, 0);
	if (IS_ERR(edge->mbox_chan))
		return dev_err_probe(dev, PTR_ERR(edge->mbox_chan),
				     "failed to acquire SPSS mailbox\n");

	size = sizeof(*cfg) + 2 * SPSS_GLINK_FIFO_SIZE;
	ret = qcom_smem_alloc(host, SPSS_GLINK_DESC_ITEM, size);
	if (ret && ret != -EEXIST)
		goto err_mbox;
	cfg = qcom_smem_get(host, SPSS_GLINK_DESC_ITEM, &size);
	if (IS_ERR(cfg)) {
		ret = PTR_ERR(cfg);
		goto err_mbox;
	}
	if (size != sizeof(*cfg) + 2 * SPSS_GLINK_FIFO_SIZE) {
		ret = -EINVAL;
		goto err_mbox;
	}
	phys = qcom_smem_virt_to_phys(cfg);
	if (!phys) {
		ret = -EINVAL;
		goto err_mbox;
	}

	cfg->tx_head = 0;
	cfg->tx_fifo_size = cpu_to_le32(SPSS_GLINK_FIFO_SIZE);
	cfg->rx_tail = 0;
	cfg->rx_fifo_size = cpu_to_le32(SPSS_GLINK_FIFO_SIZE);

	edge->tx.edge = edge;
	edge->tx.head = &cfg->tx_head;
	edge->tx.tail = &cfg->tx_tail;
	edge->tx.fifo = (u8 *)(cfg + 1);
	edge->tx.native.length = SPSS_GLINK_FIFO_SIZE;
	edge->tx.native.avail = spss_tx_avail;
	edge->tx.native.write = spss_tx_write;
	edge->tx.native.kick = spss_tx_kick;

	edge->rx.edge = edge;
	edge->rx.head = &cfg->rx_head;
	edge->rx.tail = &cfg->rx_tail;
	edge->rx.fifo = edge->tx.fifo + SPSS_GLINK_FIFO_SIZE;
	edge->rx.native.length = SPSS_GLINK_FIFO_SIZE;
	edge->rx.native.avail = spss_rx_avail;
	edge->rx.native.peek = spss_rx_peek;
	edge->rx.native.advance = spss_rx_advance;

	/* Publish the initialized descriptor before giving SPSS its address. */
	wmb();
	writel(lower_32_bits(phys), edge->handoff_addr);
	writel(upper_32_bits(phys), edge->handoff_addr + sizeof(u32));
	writel(size, edge->handoff_size);

	edge->glink_dev = kzalloc_obj(*edge->glink_dev);
	if (!edge->glink_dev) {
		ret = -ENOMEM;
		goto err_mbox;
	}
	edge->glink_dev->parent = dev;
	edge->glink_dev->of_node = dev->of_node;
	edge->glink_dev->release = spss_glink_dev_release;
	device_initialize(edge->glink_dev);
	ret = dev_set_name(edge->glink_dev, "%s:glink", dev_name(dev));
	if (ret)
		goto err_put_device;
	ret = device_add(edge->glink_dev);
	if (ret)
		goto err_put_device;

	edge->glink = qcom_glink_native_probe(edge->glink_dev,
					    GLINK_FEATURE_INTENT_REUSE,
					    &edge->rx.native, &edge->tx.native,
					    false);
	if (IS_ERR(edge->glink)) {
		ret = PTR_ERR(edge->glink);
		goto err_device;
	}

	platform_set_drvdata(pdev, edge);
	enable_irq(edge->irq);
	return 0;

err_device:
	device_unregister(edge->glink_dev);
	goto err_mbox;
err_put_device:
	put_device(edge->glink_dev);
err_mbox:
	mbox_free_channel(edge->mbox_chan);
	return ret;
}

static void spss_glink_remove(struct platform_device *pdev)
{
	struct spss_glink *edge = platform_get_drvdata(pdev);

	disable_irq(edge->irq);
	qcom_glink_native_remove(edge->glink);
	device_unregister(edge->glink_dev);
	mbox_free_channel(edge->mbox_chan);
}

static const struct of_device_id spss_glink_of_match[] = {
	{ .compatible = "qcom,hamoa-spss-glink" },
	{}
};
MODULE_DEVICE_TABLE(of, spss_glink_of_match);

static struct platform_driver spss_glink_driver = {
	.probe = spss_glink_probe,
	.remove = spss_glink_remove,
	.driver = {
		.name = "qcom-glink-spss",
		.of_match_table = spss_glink_of_match,
	},
};
module_platform_driver(spss_glink_driver);

MODULE_DESCRIPTION("Qualcomm SPSS GLINK transport");
MODULE_AUTHOR("Antheas Kapenekakis <lkml@antheas.dev>");
MODULE_LICENSE("GPL");
