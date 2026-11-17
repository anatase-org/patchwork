// SPDX-License-Identifier: GPL-2.0-only
/* TPM 2.0 client for the Qualcomm SPSS stpm_lite_msft application. */
#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/firmware/qcom/spss.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/random.h>
#include <linux/rpmsg.h>
#include <linux/sizes.h>
#include <linux/tpm.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <dt-bindings/firmware/qcom,scm.h>

#include "tpm.h"

#define SPSS_TPM_OPEN		0x7001
#define SPSS_TPM_CLOSE		0x7002
#define SPSS_TPM_SUBMIT		0x7003
#define SPSS_TPM_BUFFER_SIZE	(3 * SZ_4K)
#define SPSS_TPM_TIMEOUT		msecs_to_jiffies(120000)
#define SPSS_TPM_FRAME_SIZE	8

struct spss_tpm {
	struct device *dev;
	struct rpmsg_endpoint *endpoint;
	struct device *dma_dev;
	struct tpm_chip *chip;
	struct work_struct init_work;
	struct mutex lock;
	spinlock_t response_lock;
	struct completion done;
	void *buffer;
	dma_addr_t address;
	u64 vmids;
	u32 transaction;
	u16 command;
	int result;
	bool pending;
	bool broken;
	bool opened;
	bool registered;
};

static int spss_tpm_callback(struct rpmsg_device *rpdev, void *data, int length,
			     void *priv, u32 source)
{
	struct spss_tpm *spss = priv;
	unsigned long flags;

	if (!spss)
		return -EAGAIN;

	if (length < SPSS_TPM_FRAME_SIZE || get_unaligned_le32(data))
		return -EINVAL;

	spin_lock_irqsave(&spss->response_lock, flags);
	if (!spss->pending || get_unaligned_le32(data + 4) != spss->transaction) {
		spin_unlock_irqrestore(&spss->response_lock, flags);
		return 0;
	}

	if (length < SPSS_TPM_FRAME_SIZE + 6 ||
	    length > SPSS_TPM_FRAME_SIZE + 268 ||
	    get_unaligned_le16(data + 8) != spss->command)
		spss->result = -EPROTO;
	else if (get_unaligned_le32(data + 10)) {
		dev_err(spss->dev, "SPSS command %#x failed: %#x\n",
			spss->command, get_unaligned_le32(data + 10));
		spss->result = -EREMOTEIO;
	} else {
		spss->result = 0;
	}

	spss->pending = false;
	complete(&spss->done);
	spin_unlock_irqrestore(&spss->response_lock, flags);

	return 0;
}

static int spss_tpm_exchange(struct spss_tpm *spss, u16 command,
			     const u8 *payload, size_t size)
{
	u8 frame[SPSS_TPM_FRAME_SIZE + 18] = {};
	unsigned long flags;
	unsigned long completed;
	int ret;

	lockdep_assert_held(&spss->lock);
	spin_lock_irqsave(&spss->response_lock, flags);
	if (spss->broken && command != SPSS_TPM_CLOSE) {
		spin_unlock_irqrestore(&spss->response_lock, flags);
		return -ENOTCONN;
	}

	reinit_completion(&spss->done);
	spss->transaction++;
	spss->command = command;
	spss->pending = true;
	spss->result = -EIO;
	put_unaligned_le32(spss->transaction, frame + 4);
	spin_unlock_irqrestore(&spss->response_lock, flags);
	memcpy(frame + SPSS_TPM_FRAME_SIZE, payload, size);

	ret = rpmsg_send(spss->endpoint, frame, SPSS_TPM_FRAME_SIZE + size);
	completed = ret ? 0 : wait_for_completion_timeout(&spss->done, SPSS_TPM_TIMEOUT);
	spin_lock_irqsave(&spss->response_lock, flags);
	if (!ret)
		ret = completed ? spss->result : -ETIMEDOUT;

	/* A timed-out command may still execute. Do not reuse the transport. */
	if (ret && ret != -EREMOTEIO)
		spss->broken = true;
	spss->pending = false;
	spin_unlock_irqrestore(&spss->response_lock, flags);

	if (ret == -ETIMEDOUT)
		dev_err(spss->dev, "SPSS command %#x timed out: request=%#x status=%#x start=%#x\n",
			command, get_unaligned_le32(spss->buffer),
			get_unaligned_le32(spss->buffer + 4),
			get_unaligned_le32(spss->buffer + 12));

	return ret;
}

static int spss_tpm_open(struct spss_tpm *spss)
{
	u8 request[18] = {};
	int ret;

	put_unaligned_le16(SPSS_TPM_OPEN, request);
	put_unaligned_le32(1, request + 2);
	put_unaligned_le16(3, request + 6);
	put_unaligned_le64(spss->address, request + 8);
	put_unaligned_le16(SPSS_TPM_BUFFER_SIZE, request + 16);
	ret = spss_tpm_exchange(spss, SPSS_TPM_OPEN, request, sizeof(request));
	/* A failed exchange can leave firmware using the command buffer. */
	if (!ret || spss->broken)
		spss->opened = true;

	return ret;
}

static int spss_tpm_close(struct spss_tpm *spss)
{
	u8 request[6] = {};
	int ret;

	if (!spss->opened)
		return 0;

	put_unaligned_le16(SPSS_TPM_CLOSE, request);
	put_unaligned_le32(1, request + 2);
	ret = spss_tpm_exchange(spss, SPSS_TPM_CLOSE, request, sizeof(request));
	if (!ret)
		spss->opened = false;

	return ret;
}

static int spss_tpm_request(struct spss_tpm *spss, u32 request);

static int spss_tpm_send(struct tpm_chip *chip, u8 *buffer, size_t capacity,
			 size_t command_size)
{
	struct spss_tpm *spss = dev_get_drvdata(chip->dev.parent);
	u8 *control = spss->buffer;
	u8 *response = spss->buffer + 2 * SZ_4K;
	u8 submit[2];
	u32 response_size;
	u32 start;
	u16 tag;
	int ret;

	if (command_size < TPM_HEADER_SIZE || command_size > SZ_4K ||
	    get_unaligned_be32(buffer + 2) != command_size)
		return -EINVAL;

	guard(mutex)(&spss->lock);
	if (!spss->opened || spss->broken)
		return -ENOTCONN;

	ret = spss_tpm_request(spss, 1);
	if (ret)
		return ret;

	memcpy(spss->buffer + SZ_4K, buffer, command_size);
	memset(response, 0, TPM_HEADER_SIZE);
	put_unaligned_le32(0, control + 8);
	/* Complete the command writes before giving firmware ownership. */
	dma_wmb();
	put_unaligned_le32(1, control + 12);
	/* Publish the TPM command and start flag before ringing the SPSS doorbell. */
	dma_wmb();
	put_unaligned_le16(SPSS_TPM_SUBMIT, submit);
	dev_dbg(&chip->dev, "submit TPM %#x request=%#x status=%#x start=%#x\n",
		get_unaligned_be32(buffer + 6), get_unaligned_le32(control),
		get_unaligned_le32(control + 4), get_unaligned_le32(control + 12));
	ret = spss_tpm_exchange(spss, SPSS_TPM_SUBMIT, submit, sizeof(submit));
	if (ret)
		return ret;

	ret = read_poll_timeout(get_unaligned_le32, start, !start, 1000, 120000000,
				false, control + 12);
	if (ret) {
		spss->broken = true;
		dev_err(&chip->dev, "TPM command completion timed out: status=%#x\n",
			get_unaligned_le32(control + 4));
		return ret;
	}

	/* Observe firmware's response writes after it clears the start flag. */
	dma_rmb();
	if (get_unaligned_le32(control + 4) & BIT(0)) {
		spss->broken = true;
		return -EIO;
	}

	response_size = get_unaligned_be32(response + 2);
	tag = get_unaligned_be16(response);
	if (response_size < TPM_HEADER_SIZE || response_size > SZ_4K ||
	    response_size > capacity ||
	    (tag != TPM2_ST_NO_SESSIONS && tag != TPM2_ST_SESSIONS))
		return -EPROTO;

	dev_dbg(&chip->dev, "TPM response: status=%#x size=%u\n",
		get_unaligned_be32(response + 6), response_size);
	memcpy(buffer, response, response_size);
	ret = spss_tpm_request(spss, 2);
	if (ret)
		dev_err(&chip->dev, "SPSS idle request failed: %d\n", ret);

	return response_size;
}

static int spss_tpm_request(struct spss_tpm *spss, u32 request)
{
	u8 submit[2];
	u32 pending;
	int ret;

	lockdep_assert_held(&spss->lock);
	if (!spss->opened || spss->broken)
		return -ENOTCONN;

	if (get_unaligned_le32(spss->buffer) || get_unaligned_le32(spss->buffer + 12))
		return -EBUSY;

	/* Finish reading the response before firmware can reuse its buffer. */
	dma_mb();
	put_unaligned_le32(request, spss->buffer);
	/* Publish the ready/idle request before notifying the SPSS application. */
	dma_wmb();
	put_unaligned_le16(SPSS_TPM_SUBMIT, submit);
	ret = spss_tpm_exchange(spss, SPSS_TPM_SUBMIT, submit, sizeof(submit));
	if (ret)
		return ret;

	ret = read_poll_timeout(get_unaligned_le32, pending, !pending, 1000, 120000000,
				false, spss->buffer);
	if (ret) {
		spss->broken = true;
		dev_err(spss->dev, "SPSS request %#x timed out: status=%#x\n",
			request, get_unaligned_le32(spss->buffer + 4));
		return ret;
	}

	/* Firmware acknowledges the request by clearing it in shared memory. */
	dma_rmb();
	if (request == 1 && get_unaligned_le32(spss->buffer + 4)) {
		spss->broken = true;
		return -EIO;
	}

	return 0;
}

static const struct tpm_class_ops spss_tpm_ops = {
	.flags = TPM_OPS_AUTO_STARTUP,
	.send = spss_tpm_send,
};

static int spss_tpm_handover(struct device *dev)
{
	const struct qcom_tzmem_pool_config config = {
		.initial_size = SZ_4K,
		.policy = QCOM_TZMEM_POLICY_STATIC,
	};
	struct qcom_tzmem_pool *pool;
	void *message;
	u32 app_id;
	int ret;

	ret = qcom_scm_qseecom_app_get_id("qcom.tz.mssecapp", &app_id);
	if (ret == -ENOENT)
		return 0;
	if (ret)
		return ret;

	pool = qcom_tzmem_pool_new(&config);
	if (IS_ERR(pool))
		return PTR_ERR(pool);

	message = qcom_tzmem_alloc(pool, 128, GFP_KERNEL);
	if (!message) {
		qcom_tzmem_pool_free(pool);
		return -ENOMEM;
	}

	memset(message, 0, 128);
	put_unaligned_le32(0x4007, message);
	ret = qcom_scm_qseecom_app_send(app_id, message, 4, message + 64, 12);
	dev_dbg(dev, "mssecapp %u version query: %d %*ph\n", app_id, ret, 12, message + 64);
	/* The boot interface stops answering after it has handed over to SPSS. */
	if (!ret && !memchr_inv(message + 64, 0, 12)) {
		qcom_tzmem_free(message);
		qcom_tzmem_pool_free(pool);
		return 0;
	}

	if (!ret && (get_unaligned_le32(message + 64) ||
	    get_unaligned_le32(message + 68) != 0x4007 ||
	    get_unaligned_le32(message + 72) != 0x10000))
		ret = -EPROTO;

	if (!ret) {
		memset(message, 0, 128);
		put_unaligned_le32(0x4005, message);
		ret = qcom_scm_qseecom_app_send(app_id, message, 4, message + 64, 8);
		if (!ret && (get_unaligned_le32(message + 64) != 0x4005 ||
		    get_unaligned_le32(message + 68)))
			ret = -EIO;
	}

	qcom_tzmem_free(message);
	qcom_tzmem_pool_free(pool);
	return ret;
}

static void spss_tpm_free_buffer(void *data)
{
	struct spss_tpm *spss = data;
	const struct qcom_scm_vmperm permission = {
		.vmid = QCOM_SCM_VMID_HLOS,
		.perm = QCOM_SCM_PERM_RWX,
	};
	int ret;

	if (spss->opened) {
		dev_err(spss->dev, "locality is still open, retaining shared buffer\n");
		return;
	}

	ret = qcom_scm_assign_mem(spss->address, SPSS_TPM_BUFFER_SIZE,
				  &spss->vmids, &permission, 1);
	if (ret) {
		dev_err(spss->dev, "cannot revoke SPSS access, retaining buffer: %d\n", ret);
		return;
	}

	dma_free_coherent(spss->dma_dev, SPSS_TPM_BUFFER_SIZE, spss->buffer, spss->address);
}

static int spss_tpm_prepare_buffer(struct spss_tpm *spss)
{
	const struct qcom_scm_vmperm permissions[] = {
		{ .vmid = QCOM_SCM_VMID_HLOS, .perm = QCOM_SCM_PERM_RW },
		{ .vmid = QCOM_SCM_VMID_CP_SPSS_HLOS_SHARED, .perm = QCOM_SCM_PERM_RW },
	};
	int ret;

	if (device_iommu_mapped(spss->dma_dev))
		return -EOPNOTSUPP;

	ret = dma_set_mask_and_coherent(spss->dma_dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	spss->buffer = dma_alloc_coherent(spss->dma_dev, SPSS_TPM_BUFFER_SIZE,
					  &spss->address, GFP_KERNEL);
	if (!spss->buffer)
		return -ENOMEM;

	spss->vmids = BIT_ULL(QCOM_SCM_VMID_HLOS);
	ret = qcom_scm_assign_mem(spss->address, SPSS_TPM_BUFFER_SIZE, &spss->vmids,
				  permissions, ARRAY_SIZE(permissions));
	if (ret) {
		dma_free_coherent(spss->dma_dev, SPSS_TPM_BUFFER_SIZE, spss->buffer, spss->address);
		return ret;
	}

	put_unaligned_le32(SZ_4K, spss->buffer + 24);
	put_unaligned_le32(SZ_4K, spss->buffer + 28);
	put_unaligned_le32(SZ_4K, spss->buffer + 36);
	put_unaligned_le32(2 * SZ_4K, spss->buffer + 40);
	dev_dbg(spss->dev, "shared TPM buffer at %pad\n", &spss->address);
	return devm_add_action_or_reset(spss->dev, spss_tpm_free_buffer, spss);
}

static void spss_tpm_init_work(struct work_struct *work)
{
	struct spss_tpm *spss = container_of(work, struct spss_tpm, init_work);
	struct device *dev = spss->dev;
	int ret;

	ret = spss_tpm_handover(dev);
	if (ret) {
		dev_err(dev, "TPM firmware handoff failed: %d\n", ret);
		return;
	}

	scoped_guard(mutex, &spss->lock)
		ret = spss_tpm_open(spss);

	if (!ret)
		ret = tpm_chip_register(spss->chip);
	if (ret) {
		dev_err(dev, "TPM initialization failed: %d\n", ret);
		return;
	}

	spss->registered = true;
}

static int spss_tpm_probe(struct auxiliary_device *adev,
			  const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct spss_tpm *spss;
	int ret;

	ret = qcom_spss_nvm_link(dev);
	if (ret)
		return ret;

	if (!dev->parent || !dev->parent->parent || !dev->parent->parent->parent)
		return -ENODEV;

	spss = devm_kzalloc(dev, sizeof(*spss), GFP_KERNEL);
	if (!spss)
		return -ENOMEM;

	spss->dev = dev;
	spss->dma_dev = dev->parent->parent->parent;
	mutex_init(&spss->lock);
	spin_lock_init(&spss->response_lock);
	init_completion(&spss->done);
	INIT_WORK(&spss->init_work, spss_tpm_init_work);
	spss->transaction = get_random_u32();
	dev_set_drvdata(dev, spss);
	spss->endpoint = qcom_spss_tpm_attach(dev, spss_tpm_callback, spss);
	if (IS_ERR(spss->endpoint))
		return PTR_ERR(spss->endpoint);

	ret = devm_add_action_or_reset(dev, qcom_spss_tpm_detach, spss);
	if (ret)
		return ret;

	ret = spss_tpm_prepare_buffer(spss);
	if (ret)
		return dev_err_probe(dev, ret, "cannot share TPM command buffer\n");

	spss->chip = tpmm_chip_alloc(dev, &spss_tpm_ops);
	if (IS_ERR(spss->chip))
		return PTR_ERR(spss->chip);

	dev_set_drvdata(dev, spss);
	dev_set_drvdata(&spss->chip->dev, spss->chip);
	spss->chip->flags |= TPM_CHIP_FLAG_TPM2 | TPM_CHIP_FLAG_SYNC |
			     TPM_CHIP_FLAG_FIRMWARE_POWER_MANAGED;
	/* GLINK must be free to receive replies while initialization waits. */
	schedule_work(&spss->init_work);
	return 0;
}

static void spss_tpm_remove(struct auxiliary_device *adev)
{
	struct spss_tpm *spss = auxiliary_get_drvdata(adev);

	cancel_work_sync(&spss->init_work);
	if (spss->registered)
		tpm_chip_unregister(spss->chip);

	guard(mutex)(&spss->lock);
	if (spss->endpoint) {
		spss_tpm_close(spss);
		qcom_spss_tpm_detach(spss);
	}
}

static int spss_tpm_suspend(struct device *dev)
{
	struct spss_tpm *spss = dev_get_drvdata(dev);
	int ret;

	flush_work(&spss->init_work);
	if (!spss->registered)
		return 0;

	ret = tpm_pm_suspend(&spss->chip->dev);
	if (ret)
		return ret;

	guard(mutex)(&spss->lock);
	return spss_tpm_close(spss);
}

static int spss_tpm_resume(struct device *dev)
{
	struct spss_tpm *spss = dev_get_drvdata(dev);
	int ret;

	if (!spss->registered)
		return 0;

	guard(mutex)(&spss->chip->tpm_mutex);
	scoped_guard(mutex, &spss->lock)
		ret = spss_tpm_open(spss);

	if (ret)
		return ret;

	return tpm_pm_resume(&spss->chip->dev);
}

static DEFINE_SIMPLE_DEV_PM_OPS(spss_tpm_pm_ops, spss_tpm_suspend, spss_tpm_resume);

static const struct auxiliary_device_id spss_tpm_id_table[] = {
	{ .name = "qcom_spss_nvm.tpm" },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, spss_tpm_id_table);

static struct auxiliary_driver spss_tpm_driver = {
	.driver.name = "tpm_qcom_spss",
	.driver.pm = pm_sleep_ptr(&spss_tpm_pm_ops),
	.id_table = spss_tpm_id_table,
	.probe = spss_tpm_probe,
	.remove = spss_tpm_remove,
};
module_auxiliary_driver(spss_tpm_driver);

MODULE_DESCRIPTION("Qualcomm SPSS TPM 2.0 driver");
MODULE_AUTHOR("Antheas Kapenekakis <lkml@antheas.dev>");
MODULE_LICENSE("GPL");
