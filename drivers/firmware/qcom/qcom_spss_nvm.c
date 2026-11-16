// SPDX-License-Identifier: GPL-2.0-only
/* Secure NVM backing service for firmware-started Qualcomm SPSS applications. */
#include <crypto/sha2.h>
#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/crc32.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/spss.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/rpmsg.h>
#include <linux/sizes.h>
#include <linux/skbuff.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define NVM_SECTOR_SIZE		SZ_4K
#define NVM_SLOT_SIZE		(9 * SZ_4K)
#define NVM_MAX_BLOCKS		32
#define NVM_REQUEST_SIZE		0x87
#define NVM_FRAME_SIZE		8
#define NVM_ERROR_INVALID	4
#define NVM_ERROR_STORAGE	9

struct spss_nvm_app {
	const char *channel;
	u32 shared_offset;
	u32 store_offset;
};

static const struct spss_nvm_app nvm_apps[] = {
	{ "sec_nvm_sp_nvm", 0, 0 },
	{ "sec_nvm_stpm_lite_msft", 0x9000, 0xb000 },
	{ "sec_nvm_ksp_msft", 0x1a000, 0x1d000 },
};

struct spss_nvm_layout {
	u32 blocks;
	u32 data_size;
	u32 l2_size;
	u32 root_offset;
	u32 bitmap_offset;
	u32 counter_offset;
	u32 digest_offset;
};

struct spss_nvm {
	struct rpmsg_device *rpdev;
	struct auxiliary_device *tpm;
	const struct spss_nvm_app *app;
	struct spss_nvm_layout layout;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct sk_buff_head requests;
	void __iomem *shared;
	phys_addr_t shared_address;
	u64 partition_lba;
	u64 partition_blocks;
	u8 *slots[2];
	u8 *snapshot;
	u8 uuid[16];
	u8 async_response[18];
	bool have_uuid;
	bool async_done;
	bool stopping;
	unsigned int index;
};

static DEFINE_MUTEX(nvm_devices_lock);
static struct device *nvm_devices[ARRAY_SIZE(nvm_apps)];
static struct rpmsg_device *tpm_channel;
static DEFINE_SPINLOCK(tpm_callback_lock);
static rpmsg_rx_cb_t tpm_callback;
static void *tpm_callback_priv;

struct rpmsg_endpoint *qcom_spss_tpm_attach(struct device *consumer,
					 rpmsg_rx_cb_t callback, void *priv)
{
	unsigned long flags;

	guard(mutex)(&nvm_devices_lock);
	if (!tpm_channel)
		return ERR_PTR(-EPROBE_DEFER);

	if (!device_link_add(consumer, &tpm_channel->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return ERR_PTR(-ENOMEM);

	spin_lock_irqsave(&tpm_callback_lock, flags);
	if (tpm_callback) {
		spin_unlock_irqrestore(&tpm_callback_lock, flags);
		return ERR_PTR(-EBUSY);
	}

	tpm_callback = callback;
	tpm_callback_priv = priv;
	spin_unlock_irqrestore(&tpm_callback_lock, flags);
	return tpm_channel->ept;
}
EXPORT_SYMBOL_GPL(qcom_spss_tpm_attach);

void qcom_spss_tpm_detach(void *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&tpm_callback_lock, flags);
	if (tpm_callback_priv == priv) {
		tpm_callback = NULL;
		tpm_callback_priv = NULL;
	}
	spin_unlock_irqrestore(&tpm_callback_lock, flags);
}
EXPORT_SYMBOL_GPL(qcom_spss_tpm_detach);

static int tpm_channel_callback(struct rpmsg_device *rpdev, void *data, int length,
				void *priv, u32 source)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&tpm_callback_lock, flags);
	if (tpm_callback)
		ret = tpm_callback(rpdev, data, length, tpm_callback_priv, source);
	spin_unlock_irqrestore(&tpm_callback_lock, flags);
	return ret;
}

static int tpm_channel_probe(struct rpmsg_device *rpdev)
{
	guard(mutex)(&nvm_devices_lock);
	if (tpm_channel)
		return -EBUSY;

	tpm_channel = rpdev;
	return 0;
}

static void tpm_channel_remove(struct rpmsg_device *rpdev)
{
	guard(mutex)(&nvm_devices_lock);
	tpm_channel = NULL;
}

static const struct rpmsg_device_id tpm_channel_id_table[] = {
	{ .name = "stpm_lite_msft" },
	{}
};
MODULE_DEVICE_TABLE(rpmsg, tpm_channel_id_table);

static struct rpmsg_driver tpm_channel_driver = {
	.drv.name = "qcom_spss_tpm_channel",
	.id_table = tpm_channel_id_table,
	.probe = tpm_channel_probe,
	.callback = tpm_channel_callback,
	.remove = tpm_channel_remove,
};

int qcom_spss_nvm_link(struct device *consumer)
{
	unsigned int i;

	guard(mutex)(&nvm_devices_lock);
	for (i = 0; i < 2; i++) {
		if (!nvm_devices[i])
			return -EPROBE_DEFER;
	}

	for (i = 0; i < 2; i++) {
		if (nvm_devices[i] == consumer->parent)
			continue;

		if (!device_link_add(consumer, nvm_devices[i], DL_FLAG_AUTOREMOVE_CONSUMER))
			return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_spss_nvm_link);

static u32 nvm_crc(const void *data, size_t size)
{
	return crc32_le(~0, data, size) ^ ~0;
}

static int nvm_find_partition(struct spss_nvm *nvm)
{
	static const u8 type_guid[16] = {
		0x4c, 0x2b, 0x2e, 0xe4, 0xb0, 0x33, 0x9b, 0x42,
		0xb1, 0xef, 0xd3, 0x41, 0xc5, 0x47, 0x02, 0x2c,
	};
	u8 *header __free(kfree) = kzalloc(NVM_SECTOR_SIZE, GFP_KERNEL);
	u8 *entries __free(kfree) = NULL;
	u64 blocks, entry_lba, first, last;
	u32 block_size, crc, count, bytes, i;
	int ret;

	if (!header)
		return -ENOMEM;

	ret = qcom_scm_storage_get_geometry(&blocks, &block_size);
	if (ret)
		return ret;

	if (block_size != NVM_SECTOR_SIZE || blocks < 2)
		return -EOPNOTSUPP;

	ret = qcom_scm_storage_read(1, header, NVM_SECTOR_SIZE);
	if (ret)
		return ret;

	if (memcmp(header, "EFI PART", 8) || get_unaligned_le32(header + 12) != 92)
		return -EINVAL;

	crc = get_unaligned_le32(header + 16);
	put_unaligned_le32(0, header + 16);
	if (nvm_crc(header, 92) != crc)
		return -EBADMSG;

	entry_lba = get_unaligned_le64(header + 72);
	count = get_unaligned_le32(header + 80);
	if (!count || count > 128 || get_unaligned_le32(header + 84) != 128)
		return -EINVAL;

	bytes = ALIGN(count * 128, NVM_SECTOR_SIZE);
	if (entry_lba >= blocks || bytes / NVM_SECTOR_SIZE > blocks - entry_lba)
		return -EINVAL;

	entries = kmalloc(bytes, GFP_KERNEL);
	if (!entries)
		return -ENOMEM;

	for (i = 0; i < bytes; i += NVM_SECTOR_SIZE) {
		ret = qcom_scm_storage_read(entry_lba + i / NVM_SECTOR_SIZE,
					    entries + i, NVM_SECTOR_SIZE);
		if (ret)
			return ret;
	}

	if (nvm_crc(entries, count * 128) != get_unaligned_le32(header + 88))
		return -EBADMSG;

	for (i = 0; i < count; i++) {
		const u8 *entry = entries + i * 128;

		if (memcmp(entry, type_guid, sizeof(type_guid)))
			continue;

		first = get_unaligned_le64(entry + 32);
		last = get_unaligned_le64(entry + 40);
		if (first > last || last >= blocks || first < 6)
			return -EINVAL;

		nvm->partition_lba = first;
		nvm->partition_blocks = last - first + 1;
		return 0;
	}

	return -ENOENT;
}

/* The SP NVM backup shares its seven middle sectors with the primary. */
static u32 nvm_sector_offset(struct spss_nvm *nvm, unsigned int slot, u32 offset)
{
	if (!nvm->index && slot) {
		if (!offset)
			return 0x9000;
		if (offset == 0x8000)
			return 0xa000;
		return offset;
	}

	return nvm->app->store_offset + slot * NVM_SLOT_SIZE + offset;
}

static int nvm_read_slot(struct spss_nvm *nvm, unsigned int slot, u8 *data)
{
	u32 offset;
	int ret;

	for (offset = 0; offset < NVM_SLOT_SIZE; offset += NVM_SECTOR_SIZE) {
		u64 sector = nvm_sector_offset(nvm, slot, offset) / NVM_SECTOR_SIZE;

		if (sector >= nvm->partition_blocks)
			return -EINVAL;

		ret = qcom_scm_storage_read(nvm->partition_lba + sector,
					    data + offset, NVM_SECTOR_SIZE);
		if (ret)
			return ret;
	}

	return 0;
}

static struct spss_nvm_layout nvm_layout(u32 blocks)
{
	struct spss_nvm_layout layout = {
		.blocks = blocks,
		.data_size = blocks * SZ_1K,
		.l2_size = blocks / 32 * 0x600,
	};

	layout.root_offset = layout.data_size + layout.l2_size + blocks / 32 * 32 + 0x30;
	layout.bitmap_offset = layout.root_offset + 16;
	layout.counter_offset = layout.bitmap_offset + blocks;
	layout.digest_offset = layout.counter_offset + 4;
	return layout;
}

static bool nvm_valid_slot(const struct spss_nvm_layout *layout, const u8 *data)
{
	u8 digest[SHA256_DIGEST_SIZE];

	sha256(data, layout->digest_offset, digest);
	return !memcmp(digest, data + layout->digest_offset, sizeof(digest));
}

static int nvm_select_slot(struct spss_nvm *nvm, const u8 *marker, bool skp)
{
	unsigned int i, marker_offset = nvm->layout.root_offset;
	size_t marker_size = 16;
	u32 counter = 0;
	int selected = -EBADMSG;

	if (skp) {
		marker_offset -= 32;
		marker_size = 32;
	}

	for (i = 0; i < 2; i++) {
		u32 slot_counter = get_unaligned_le32(nvm->slots[i] + nvm->layout.counter_offset);

		if (nvm_valid_slot(&nvm->layout, nvm->slots[i]) &&
		    !memcmp(nvm->slots[i] + marker_offset, marker, marker_size) &&
		    (selected < 0 || slot_counter > counter)) {
			selected = i;
			counter = slot_counter;
		}
	}

	return selected;
}

static int nvm_write_slot(struct spss_nvm *nvm, unsigned int slot, u8 *data)
{
	u32 offset;
	int ret;

	/* Commit the sector containing the marker, counter and digest last. */
	for (offset = 0; offset < NVM_SLOT_SIZE; offset += NVM_SECTOR_SIZE) {
		u64 sector = nvm_sector_offset(nvm, slot, offset) / NVM_SECTOR_SIZE;

		if (sector >= nvm->partition_blocks)
			return -EINVAL;

		if (!memcmp(nvm->slots[slot] + offset, data + offset, NVM_SECTOR_SIZE))
			continue;

		ret = qcom_scm_storage_write(nvm->partition_lba + sector,
					     data + offset, NVM_SECTOR_SIZE);
		if (ret)
			return ret;
	}

	ret = nvm_read_slot(nvm, slot, nvm->slots[slot]);
	if (ret)
		return ret;

	if (memcmp(data, nvm->slots[slot], NVM_SLOT_SIZE))
		return -EIO;

	return 0;
}

static int nvm_commit(struct spss_nvm *nvm, const u8 *request, u8 *response)
{
	const struct spss_nvm_layout *layout = &nvm->layout;
	u32 i, counter;
	bool skp = request[0] == 8;
	const u8 *marker = skp ? request + 0x67 : request + 0x13;
	u8 *data = nvm->snapshot;
	u8 bitmap[NVM_MAX_BLOCKS];
	int active, ret;

	if (request[0x11] != 1 || request[0x12])
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		ret = nvm_read_slot(nvm, i, nvm->slots[i]);
		if (ret)
			return ret;
	}

	active = nvm_select_slot(nvm, marker, skp);
	if (active < 0)
		return active;

	counter = get_unaligned_le32(nvm->slots[active] + layout->counter_offset);
	if (counter == U32_MAX)
		return -EOVERFLOW;

	memcpy(data, nvm->slots[active], NVM_SLOT_SIZE);
	memcpy(data + layout->digest_offset + SHA256_DIGEST_SIZE,
	       nvm->slots[active ^ 1] + layout->digest_offset + SHA256_DIGEST_SIZE,
	       NVM_SLOT_SIZE - layout->digest_offset - SHA256_DIGEST_SIZE);
	memcpy_fromio(bitmap, nvm->shared + layout->bitmap_offset + 2, layout->blocks);
	for (i = 0; i < layout->blocks; i++) {
		if (!bitmap[i])
			continue;

		memcpy_fromio(data + i * SZ_1K, nvm->shared + i * SZ_1K, SZ_1K);
		memcpy_fromio(data + layout->data_size + i * 0x30,
			      nvm->shared + layout->data_size + i * 0x30, 0x30);
	}

	memcpy_fromio(data + layout->data_size + layout->l2_size,
		      nvm->shared + layout->data_size + layout->l2_size,
		      layout->bitmap_offset - layout->data_size - layout->l2_size);
	memcpy(data + layout->bitmap_offset, bitmap, layout->blocks);
	put_unaligned_le32(counter + 1, data + layout->counter_offset);
	sha256(data, layout->digest_offset, data + layout->digest_offset);
	ret = nvm_write_slot(nvm, active ^ 1, data);
	if (ret)
		return ret;

	dev_dbg(&nvm->rpdev->dev, "committed secure NVM backup %u\n", active ^ 1);

	if (skp)
		memcpy(response + 0x12, data + layout->root_offset - 32, 32);

	return 0;
}

static int nvm_load(struct spss_nvm *nvm, const u8 *marker, bool skp)
{
	const struct spss_nvm_layout *layout = &nvm->layout;
	unsigned int i;
	int active, ret;

	for (i = 0; i < 2; i++) {
		ret = nvm_read_slot(nvm, i, nvm->slots[i]);
		if (ret)
			return ret;
	}

	active = nvm_select_slot(nvm, marker, skp);
	if (active < 0)
		return active;

	memcpy_toio(nvm->shared, nvm->slots[active], layout->bitmap_offset);
	memset_io(nvm->shared + layout->bitmap_offset, 0, 2 + layout->blocks);
	/* Complete the shared-memory update before replying to SPSS. */
	wmb();
	return 0;
}

static int nvm_dispatch(struct spss_nvm *nvm, const u8 *request, size_t size,
			u8 *response)
{
	u8 operation = request[0];
	bool skp = operation == 7 || operation == 12;
	int ret, response_size = 0x12;

	dev_dbg(&nvm->rpdev->dev, "NVM request %u, %zu bytes\n", operation, size);

	if (operation == 13 || operation == 15) {
		response[0] = 0x6b;
		return 2;
	}

	if (operation == 14 || operation == 16) {
		response[0] = 0x6c;
		response[1] = NVM_ERROR_INVALID;
		return 2;
	}

	/* The firmware owns provisioning of these pre-existing partitions. */
	if (operation == 0 || operation == 6 || operation == 9 || operation == 11) {
		response[0] = 0x64;
		if (size >= 0x12) {
			memcpy(response + 1, request + 1, 16);
			response[0x11] = request[0x11];
		}
		response[0x2e] = NVM_ERROR_INVALID;
		return 0x2f;
	}

	if (operation == 5) {
		response[0] = 0x69;
		if (size < 2 || request[1] < 3 || request[1] > 5) {
			response[0x14] = size < 2 ? NVM_ERROR_INVALID : 10;
			return 0x15;
		}

		response[0x13] = nvm->async_done;
		if (nvm->async_done) {
			memcpy(response + 1, nvm->async_response, sizeof(nvm->async_response));
			nvm->async_done = false;
		}
		return 0x15;
	}

	response[0] = operation + 100;
	if (size < 0x12 || request[0x11] != 1) {
		response[0x11] = NVM_ERROR_INVALID;
		return response_size;
	}

	if (nvm->have_uuid && memcmp(nvm->uuid, request + 1, sizeof(nvm->uuid))) {
		response[0x11] = NVM_ERROR_INVALID;
		return response_size;
	}

	switch (operation) {
	case 1:
	case 7:
	case 10:
	case 12:
		response[0] = 0x65;
		memcpy(response + 1, request + 1, 16);
		response[0x11] = 1;
		put_unaligned_le64(nvm->shared_address, response + 0x12);
		response_size = 0x2f;
		if (size < 0x82 || ((operation == 10 || operation == 12) &&
		    (size < 0x96 || get_unaligned_le32(request + 0x82) != nvm->layout.blocks))) {
			response[0x2e] = NVM_ERROR_INVALID;
			return response_size;
		}

		ret = nvm_load(nvm, request + (skp ? 0x12 : 0x32), skp);
		response[0x2e] = ret ? NVM_ERROR_STORAGE : 0;
		break;
	case 2:
	case 4:
	case 8:
		response[0] = 0x66;
		response_size = 0x32;
		ret = size == NVM_REQUEST_SIZE ? nvm_commit(nvm, request, response) : -EINVAL;
		response[0x11] = ret ? NVM_ERROR_STORAGE : 0;
		if (operation == 4) {
			memcpy(nvm->async_response, response, sizeof(nvm->async_response));
			nvm->async_done = true;
			response[0] = 0x68;
			response[1] = 0;
			response_size = 2;
		}
		break;
	case 3:
		response[0] = 0x67;
		ret = size >= 0x67 && !request[0x12] ? nvm_load(nvm, request + 0x13, false) : -EINVAL;
		response[0x11] = ret ? NVM_ERROR_STORAGE : 0;
		break;
	default:
		ret = -EOPNOTSUPP;
		response[0x11] = NVM_ERROR_INVALID;
		break;
	}

	if (ret) {
		dev_err_ratelimited(&nvm->rpdev->dev, "NVM operation %u failed: %d\n", operation, ret);
	} else if (!nvm->have_uuid) {
		memcpy(nvm->uuid, request + 1, sizeof(nvm->uuid));
		nvm->have_uuid = true;
	}

	return response_size;
}

static void nvm_work(struct work_struct *work)
{
	struct spss_nvm *nvm = container_of(work, struct spss_nvm, work);
	struct sk_buff *skb;
	u8 response[NVM_FRAME_SIZE + 0x32];
	int size, ret;

	while ((skb = skb_dequeue(&nvm->requests))) {
		memset(response, 0, sizeof(response));
		memcpy(response + 4, skb->data + 4, 4);
		size = nvm_dispatch(nvm, skb->data + NVM_FRAME_SIZE,
				    skb->len - NVM_FRAME_SIZE, response + NVM_FRAME_SIZE);
		ret = rpmsg_send(nvm->rpdev->ept, response, size + NVM_FRAME_SIZE);
		if (ret)
			dev_err_ratelimited(&nvm->rpdev->dev, "NVM response failed: %d\n", ret);
		kfree_skb(skb);
	}
}

static int nvm_callback(struct rpmsg_device *rpdev, void *data, int length,
			void *priv, u32 source)
{
	struct spss_nvm *nvm = dev_get_drvdata(&rpdev->dev);
	struct sk_buff *skb;

	if (!nvm)
		return -EAGAIN;

	if (length <= NVM_FRAME_SIZE || length > NVM_FRAME_SIZE + 0x106 ||
	    get_unaligned_le32(data))
		return -EINVAL;

	skb = alloc_skb(length, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, data, length);
	guard(spinlock_irqsave)(&nvm->requests.lock);
	if (nvm->stopping || nvm->requests.qlen >= 8) {
		kfree_skb(skb);
		return -ESHUTDOWN;
	}

	__skb_queue_tail(&nvm->requests, skb);
	queue_work(nvm->workqueue, &nvm->work);
	return 0;
}

static void nvm_tpm_release(struct device *dev)
{
	kfree(to_auxiliary_dev(dev));
}

static int nvm_add_tpm(struct spss_nvm *nvm)
{
	struct auxiliary_device *tpm;
	int ret;

	tpm = kzalloc_obj(*tpm);
	if (!tpm)
		return -ENOMEM;

	tpm->name = "tpm";
	tpm->dev.parent = &nvm->rpdev->dev;
	tpm->dev.release = nvm_tpm_release;
	ret = auxiliary_device_init(tpm);
	if (ret) {
		kfree(tpm);
		return ret;
	}

	ret = auxiliary_device_add(tpm);
	if (ret) {
		auxiliary_device_uninit(tpm);
		return ret;
	}

	nvm->tpm = tpm;
	return 0;
}

static int nvm_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct device_node *node;
	struct resource resource;
	struct spss_nvm *nvm;
	unsigned int i;
	int ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	for (i = 0; i < ARRAY_SIZE(nvm_apps); i++) {
		if (!strcmp(rpdev->id.name, nvm_apps[i].channel))
			break;
	}
	if (i == ARRAY_SIZE(nvm_apps))
		return -ENODEV;

	nvm = devm_kzalloc(dev, sizeof(*nvm), GFP_KERNEL);
	if (!nvm)
		return -ENOMEM;

	nvm->rpdev = rpdev;
	nvm->index = i;
	nvm->app = &nvm_apps[i];
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node)
		return -EINVAL;

	ret = of_address_to_resource(node, 0, &resource);
	of_node_put(node);
	if (ret)
		return ret;

	if (resource_size(&resource) < nvm->app->shared_offset + NVM_SLOT_SIZE)
		return -EINVAL;

	resource.start += nvm->app->shared_offset;
	resource.end = resource.start + NVM_SLOT_SIZE - 1;
	nvm->shared_address = resource.start;
	nvm->shared = devm_ioremap_resource(dev, &resource);
	if (IS_ERR(nvm->shared))
		return PTR_ERR(nvm->shared);

	ret = nvm_find_partition(nvm);
	if (ret)
		return dev_err_probe(dev, ret, "cannot locate secure NVM partition\n");

	for (i = 0; i < 2; i++) {
		nvm->slots[i] = devm_kmalloc(dev, NVM_SLOT_SIZE, GFP_KERNEL);
		if (!nvm->slots[i])
			return -ENOMEM;

		ret = nvm_read_slot(nvm, i, nvm->slots[i]);
		if (ret)
			return dev_err_probe(dev, ret, "cannot read secure NVM\n");
	}

	nvm->layout = nvm_layout(NVM_MAX_BLOCKS);
	if (!nvm_valid_slot(&nvm->layout, nvm->slots[0]) &&
	    !nvm_valid_slot(&nvm->layout, nvm->slots[1]))
		return dev_err_probe(dev, -EBADMSG, "no valid secure NVM backup\n");

	nvm->snapshot = devm_kmalloc(dev, NVM_SLOT_SIZE, GFP_KERNEL);
	if (!nvm->snapshot)
		return -ENOMEM;

	skb_queue_head_init(&nvm->requests);
	INIT_WORK(&nvm->work, nvm_work);
	nvm->workqueue = alloc_ordered_workqueue("spss_nvm", WQ_MEM_RECLAIM);
	if (!nvm->workqueue)
		return -ENOMEM;

	dev_set_drvdata(dev, nvm);
	scoped_guard(mutex, &nvm_devices_lock)
		nvm_devices[nvm->index] = dev;

	if (nvm->index == 1) {
		ret = nvm_add_tpm(nvm);
		if (ret) {
			scoped_guard(mutex, &nvm_devices_lock)
				nvm_devices[nvm->index] = NULL;

			destroy_workqueue(nvm->workqueue);
			return ret;
		}
	}

	return 0;
}

static void nvm_remove(struct rpmsg_device *rpdev)
{
	struct spss_nvm *nvm = dev_get_drvdata(&rpdev->dev);

	if (nvm->tpm) {
		auxiliary_device_delete(nvm->tpm);
		auxiliary_device_uninit(nvm->tpm);
	}

	scoped_guard(mutex, &nvm_devices_lock)
		nvm_devices[nvm->index] = NULL;

	scoped_guard(spinlock_irqsave, &nvm->requests.lock)
		nvm->stopping = true;

	destroy_workqueue(nvm->workqueue);
	skb_queue_purge(&nvm->requests);
}

static const struct rpmsg_device_id nvm_id_table[] = {
	{ .name = "sec_nvm_sp_nvm" },
	{ .name = "sec_nvm_stpm_lite_msft" },
	{ .name = "sec_nvm_ksp_msft" },
	{}
};
MODULE_DEVICE_TABLE(rpmsg, nvm_id_table);

static const struct of_device_id nvm_of_match[] = {
	{ .compatible = "qcom,hamoa-spss-nvm" },
	{}
};
MODULE_DEVICE_TABLE(of, nvm_of_match);

static struct rpmsg_driver nvm_driver = {
	.drv.name = "qcom_spss_nvm",
	.drv.of_match_table = nvm_of_match,
	.id_table = nvm_id_table,
	.probe = nvm_probe,
	.callback = nvm_callback,
	.remove = nvm_remove,
};

static int __init spss_nvm_init(void)
{
	int ret;

	ret = register_rpmsg_driver(&tpm_channel_driver);
	if (ret)
		return ret;

	ret = register_rpmsg_driver(&nvm_driver);
	if (ret)
		unregister_rpmsg_driver(&tpm_channel_driver);

	return ret;
}
module_init(spss_nvm_init);

static void __exit spss_nvm_exit(void)
{
	unregister_rpmsg_driver(&nvm_driver);
	unregister_rpmsg_driver(&tpm_channel_driver);
}
module_exit(spss_nvm_exit);

MODULE_DESCRIPTION("Qualcomm SPSS secure NVM backing service");
MODULE_AUTHOR("Antheas Kapenekakis <lkml@antheas.dev>");
MODULE_LICENSE("GPL");
