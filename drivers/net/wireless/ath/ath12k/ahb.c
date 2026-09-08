// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/dma-mapping.h>
#include <linux/firmware/qcom/qcom_pas.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/bitfield.h>
#include <linux/soc/qcom/qcom_aoss.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/of_reserved_mem.h>
#include "ahb.h"
#include "debug.h"
#include "ce.h"
#include "hif.h"
#include "pci.h"		/* WCN7750 reuses the shared MSI/CE IRQ helpers */

#define ATH12K_IRQ_CE0_OFFSET 4
#define ATH12K_UPD_IRQ_WRD_LEN  18

static struct ath12k_ahb_driver *ath12k_ahb_family_drivers[ATH12K_DEVICE_FAMILY_MAX];
static const char ath12k_userpd_irq[][9] = {"spawn",
				     "ready",
				     "stop-ack"};

/*
 * Multi-UserPD Architecture:
 *
 * One Q6 RootPD (managed by separate rproc driver) supports multiple
 * ath12k UserPDs. Each UserPD represents a WiFi radio instance.
 *
 * Lifecycle:
 *  - RootPD boots when first UserPD probes
 *  - All UserPDs share RootPD's SSR notifier
 *
 * Locking:
 *  - ath12k_rproc_info_lock: Protects g_rproc_info allocation/free
 */
static struct ath12k_ahb_rproc_info *g_rproc_info;
static DEFINE_MUTEX(ath12k_rproc_info_lock);

static const char *irq_name[ATH12K_IRQ_NUM_MAX] = {
	"misc-pulse1",
	"misc-latch",
	"sw-exception",
	"watchdog",
	"ce0",
	"ce1",
	"ce2",
	"ce3",
	"ce4",
	"ce5",
	"ce6",
	"ce7",
	"ce8",
	"ce9",
	"ce10",
	"ce11",
	"host2wbm-desc-feed",
	"host2reo-re-injection",
	"host2reo-command",
	"host2rxdma-monitor-ring3",
	"host2rxdma-monitor-ring2",
	"host2rxdma-monitor-ring1",
	"reo2ost-exception",
	"wbm2host-rx-release",
	"reo2host-status",
	"reo2host-destination-ring4",
	"reo2host-destination-ring3",
	"reo2host-destination-ring2",
	"reo2host-destination-ring1",
	"rxdma2host-monitor-destination-mac3",
	"rxdma2host-monitor-destination-mac2",
	"rxdma2host-monitor-destination-mac1",
	"ppdu-end-interrupts-mac3",
	"ppdu-end-interrupts-mac2",
	"ppdu-end-interrupts-mac1",
	"rxdma2host-monitor-status-ring-mac3",
	"rxdma2host-monitor-status-ring-mac2",
	"rxdma2host-monitor-status-ring-mac1",
	"host2rxdma-host-buf-ring-mac3",
	"host2rxdma-host-buf-ring-mac2",
	"host2rxdma-host-buf-ring-mac1",
	"rxdma2host-destination-ring-mac3",
	"rxdma2host-destination-ring-mac2",
	"rxdma2host-destination-ring-mac1",
	"host2tcl-input-ring4",
	"host2tcl-input-ring3",
	"host2tcl-input-ring2",
	"host2tcl-input-ring1",
	"wbm2host-tx-completions-ring4",
	"wbm2host-tx-completions-ring3",
	"wbm2host-tx-completions-ring2",
	"wbm2host-tx-completions-ring1",
	"tcl2host-status-ring",
};

enum ext_irq_num {
	host2wbm_desc_feed = 16,
	host2reo_re_injection,
	host2reo_command,
	host2rxdma_monitor_ring3,
	host2rxdma_monitor_ring2,
	host2rxdma_monitor_ring1,
	reo2host_exception,
	wbm2host_rx_release,
	reo2host_status,
	reo2host_destination_ring4,
	reo2host_destination_ring3,
	reo2host_destination_ring2,
	reo2host_destination_ring1,
	rxdma2host_monitor_destination_mac3,
	rxdma2host_monitor_destination_mac2,
	rxdma2host_monitor_destination_mac1,
	ppdu_end_interrupts_mac3,
	ppdu_end_interrupts_mac2,
	ppdu_end_interrupts_mac1,
	rxdma2host_monitor_status_ring_mac3,
	rxdma2host_monitor_status_ring_mac2,
	rxdma2host_monitor_status_ring_mac1,
	host2rxdma_host_buf_ring_mac3,
	host2rxdma_host_buf_ring_mac2,
	host2rxdma_host_buf_ring_mac1,
	rxdma2host_destination_ring_mac3,
	rxdma2host_destination_ring_mac2,
	rxdma2host_destination_ring_mac1,
	host2tcl_input_ring4,
	host2tcl_input_ring3,
	host2tcl_input_ring2,
	host2tcl_input_ring1,
	wbm2host_tx_completions_ring4,
	wbm2host_tx_completions_ring3,
	wbm2host_tx_completions_ring2,
	wbm2host_tx_completions_ring1,
	tcl2host_status_ring,
};

static u32 ath12k_ahb_read32(struct ath12k_base *ab, u32 offset)
{
	if (ab->ce_remap && offset < ab->cmem_offset)
		return ioread32(ab->mem_ce + offset);
	return ioread32(ab->mem + offset);
}

static void ath12k_ahb_write32(struct ath12k_base *ab, u32 offset,
			       u32 value)
{
	if (ab->ce_remap && offset < ab->cmem_offset)
		iowrite32(value, ab->mem_ce + offset);
	else
		iowrite32(value, ab->mem + offset);
}

static void ath12k_ahb_cancel_workqueue(struct ath12k_base *ab)
{
	int i;

	for (i = 0; i < ab->hw_params->ce_count; i++) {
		struct ath12k_ce_pipe *ce_pipe = &ab->ce.ce_pipe[i];

		if (ath12k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR)
			continue;

		cancel_work_sync(&ce_pipe->intr_wq);
	}
}

static void ath12k_ahb_ext_grp_disable(struct ath12k_ext_irq_grp *irq_grp)
{
	int i;

	for (i = 0; i < irq_grp->num_irq; i++)
		disable_irq_nosync(irq_grp->ab->irq_num[irq_grp->irqs[i]]);
}

static void __ath12k_ahb_ext_irq_disable(struct ath12k_base *ab)
{
	int i;

	for (i = 0; i < ATH12K_EXT_IRQ_GRP_NUM_MAX; i++) {
		struct ath12k_ext_irq_grp *irq_grp = &ab->ext_irq_grp[i];

		ath12k_ahb_ext_grp_disable(irq_grp);
		if (irq_grp->napi_enabled) {
			napi_synchronize(&irq_grp->napi);
			napi_disable(&irq_grp->napi);
			irq_grp->napi_enabled = false;
		}
	}
}

static void ath12k_ahb_ext_grp_enable(struct ath12k_ext_irq_grp *irq_grp)
{
	int i;

	for (i = 0; i < irq_grp->num_irq; i++)
		enable_irq(irq_grp->ab->irq_num[irq_grp->irqs[i]]);
}

static void ath12k_ahb_setbit32(struct ath12k_base *ab, u8 bit, u32 offset)
{
	u32 val;

	val = ath12k_ahb_read32(ab, offset);
	ath12k_ahb_write32(ab, offset, val | BIT(bit));
}

static void ath12k_ahb_clearbit32(struct ath12k_base *ab, u8 bit, u32 offset)
{
	u32 val;

	val = ath12k_ahb_read32(ab, offset);
	ath12k_ahb_write32(ab, offset, val & ~BIT(bit));
}

static void ath12k_ahb_ce_irq_enable(struct ath12k_base *ab, u16 ce_id)
{
	const struct ce_attr *ce_attr;
	const struct ce_ie_addr *ce_ie_addr = ab->hw_params->ce_ie_addr;
	u32 ie1_reg_addr, ie2_reg_addr, ie3_reg_addr;

	ie1_reg_addr = ce_ie_addr->ie1_reg_addr;
	ie2_reg_addr = ce_ie_addr->ie2_reg_addr;
	ie3_reg_addr = ce_ie_addr->ie3_reg_addr;

	ce_attr = &ab->hw_params->host_ce_config[ce_id];
	if (ce_attr->src_nentries)
		ath12k_ahb_setbit32(ab, ce_id, ie1_reg_addr);

	if (ce_attr->dest_nentries) {
		ath12k_ahb_setbit32(ab, ce_id, ie2_reg_addr);
		ath12k_ahb_setbit32(ab, ce_id + CE_HOST_IE_3_SHIFT,
				    ie3_reg_addr);
	}
}

static void ath12k_ahb_ce_irq_disable(struct ath12k_base *ab, u16 ce_id)
{
	const struct ce_attr *ce_attr;
	const struct ce_ie_addr *ce_ie_addr = ab->hw_params->ce_ie_addr;
	u32 ie1_reg_addr, ie2_reg_addr, ie3_reg_addr;

	ie1_reg_addr = ce_ie_addr->ie1_reg_addr;
	ie2_reg_addr = ce_ie_addr->ie2_reg_addr;
	ie3_reg_addr = ce_ie_addr->ie3_reg_addr;

	ce_attr = &ab->hw_params->host_ce_config[ce_id];
	if (ce_attr->src_nentries)
		ath12k_ahb_clearbit32(ab, ce_id, ie1_reg_addr);

	if (ce_attr->dest_nentries) {
		ath12k_ahb_clearbit32(ab, ce_id, ie2_reg_addr);
		ath12k_ahb_clearbit32(ab, ce_id + CE_HOST_IE_3_SHIFT,
				      ie3_reg_addr);
	}
}

static void ath12k_ahb_sync_ce_irqs(struct ath12k_base *ab)
{
	int i;
	int irq_idx;

	for (i = 0; i < ab->hw_params->ce_count; i++) {
		if (ath12k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR)
			continue;

		irq_idx = ATH12K_IRQ_CE0_OFFSET + i;
		synchronize_irq(ab->irq_num[irq_idx]);
	}
}

static void ath12k_ahb_sync_ext_irqs(struct ath12k_base *ab)
{
	int i, j;
	int irq_idx;

	for (i = 0; i < ATH12K_EXT_IRQ_GRP_NUM_MAX; i++) {
		struct ath12k_ext_irq_grp *irq_grp = &ab->ext_irq_grp[i];

		for (j = 0; j < irq_grp->num_irq; j++) {
			irq_idx = irq_grp->irqs[j];
			synchronize_irq(ab->irq_num[irq_idx]);
		}
	}
}

static void ath12k_ahb_ce_irqs_enable(struct ath12k_base *ab)
{
	int i;

	for (i = 0; i < ab->hw_params->ce_count; i++) {
		if (ath12k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR)
			continue;
		ath12k_ahb_ce_irq_enable(ab, i);
	}
}

static void ath12k_ahb_ce_irqs_disable(struct ath12k_base *ab)
{
	int i;

	for (i = 0; i < ab->hw_params->ce_count; i++) {
		if (ath12k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR)
			continue;
		ath12k_ahb_ce_irq_disable(ab, i);
	}
}

static int ath12k_ahb_start(struct ath12k_base *ab)
{
	ath12k_ahb_ce_irqs_enable(ab);
	ath12k_ce_rx_post_buf(ab);

	return 0;
}

static void ath12k_ahb_ext_irq_enable(struct ath12k_base *ab)
{
	struct ath12k_ext_irq_grp *irq_grp;
	int i;

	for (i = 0; i < ATH12K_EXT_IRQ_GRP_NUM_MAX; i++) {
		irq_grp = &ab->ext_irq_grp[i];
		if (!irq_grp->napi_enabled) {
			napi_enable(&irq_grp->napi);
			irq_grp->napi_enabled = true;
		}
		ath12k_ahb_ext_grp_enable(irq_grp);
	}
}

static void ath12k_ahb_ext_irq_disable(struct ath12k_base *ab)
{
	__ath12k_ahb_ext_irq_disable(ab);
	ath12k_ahb_sync_ext_irqs(ab);
}

static void ath12k_ahb_stop(struct ath12k_base *ab)
{
	if (!test_bit(ATH12K_FLAG_CRASH_FLUSH, &ab->dev_flags))
		ath12k_ahb_ce_irqs_disable(ab);
	ath12k_ahb_sync_ce_irqs(ab);
	ath12k_ahb_cancel_workqueue(ab);
	timer_delete_sync(&ab->rx_replenish_retry);
	ath12k_ce_cleanup_pipes(ab);
}

static int ath12k_ahb_power_up(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	char fw_name[ATH12K_USERPD_FW_NAME_LEN];
	char fw2_name[ATH12K_USERPD_FW_NAME_LEN];
	struct device *dev = ab->dev;
	const struct firmware *fw, *fw2;
	unsigned long time_left;
	phys_addr_t mem_phys;
	struct resource res;
	void *mem_region;
	size_t mem_size;
	u32 pasid;
	int ret;

	ret = of_reserved_mem_region_to_resource_byname(dev->of_node, "q6-region",
							&res);
	if (ret)
		return ret;

	mem_phys = res.start;
	mem_size = resource_size(&res);
	mem_region = devm_memremap(dev, mem_phys, mem_size, MEMREMAP_WC);
	if (IS_ERR(mem_region)) {
		ath12k_err(ab, "unable to map memory region: %pa+%zx\n",
			   &res.start, mem_size);
		return PTR_ERR(mem_region);
	}

	snprintf(fw_name, sizeof(fw_name), "%s/%s/%s%d%s", ATH12K_FW_DIR,
		 ab->hw_params->fw.dir, ATH12K_AHB_FW_PREFIX, ab_ahb->userpd_id,
		 ATH12K_AHB_FW_SUFFIX);

	ret = request_firmware(&fw, fw_name, dev);
	if (ret < 0) {
		ath12k_err(ab, "request_firmware failed\n");
		return ret;
	}

	ath12k_dbg(ab, ATH12K_DBG_AHB, "Booting fw image %s, size %zd\n", fw_name,
		   fw->size);

	if (!fw->size) {
		ath12k_err(ab, "Invalid firmware size\n");
		ret = -EINVAL;
		goto err_fw;
	}

	pasid = (u32_encode_bits(ab_ahb->userpd_id, ATH12K_USERPD_ID_MASK)) |
		ATH12K_AHB_UPD_SWID;

	/* Load FW image to a reserved memory location */
	if (ab_ahb->scm_auth_enabled)
		ret = qcom_mdt_load(dev, fw, fw_name, pasid, mem_region,
				    mem_phys, mem_size, &mem_phys);
	else
		ret = qcom_mdt_load_no_init(dev, fw, fw_name, mem_region,
					    mem_phys, mem_size, &mem_phys);
	if (ret) {
		ath12k_err(ab, "Failed to load MDT segments: %d\n", ret);
		goto err_fw;
	}

	snprintf(fw2_name, sizeof(fw2_name), "%s/%s/%s", ATH12K_FW_DIR,
		 ab->hw_params->fw.dir, ATH12K_AHB_FW2);

	ret = request_firmware(&fw2, fw2_name, dev);
	if (ret < 0) {
		ath12k_err(ab, "request_firmware failed\n");
		goto err_fw;
	}

	ath12k_dbg(ab, ATH12K_DBG_AHB, "Booting fw image %s, size %zd\n", fw2_name,
		   fw2->size);

	if (!fw2->size) {
		ath12k_err(ab, "Invalid firmware size\n");
		ret = -EINVAL;
		goto err_fw2;
	}

	ret = qcom_mdt_load_no_init(dev, fw2, fw2_name, mem_region, mem_phys,
				    mem_size, &mem_phys);
	if (ret) {
		ath12k_err(ab, "Failed to load MDT segments: %d\n", ret);
		goto err_fw2;
	}

	if (ab_ahb->scm_auth_enabled) {
		/* Authenticate FW image using peripheral ID */
		ret = qcom_pas_auth_and_reset(pasid);
		if (ret) {
			ath12k_err(ab, "failed to boot the remote processor %d\n", ret);
			goto err_fw2;
		}
	}

	/* Instruct Q6 to spawn userPD thread */
	ret = qcom_smem_state_update_bits(ab_ahb->spawn_state, BIT(ab_ahb->spawn_bit),
					  BIT(ab_ahb->spawn_bit));
	if (ret) {
		ath12k_err(ab, "Failed to update spawn state %d\n", ret);
		goto err_fw2;
	}

	time_left = wait_for_completion_timeout(&ab_ahb->userpd_spawned,
						ATH12K_USERPD_SPAWN_TIMEOUT);
	if (!time_left) {
		ath12k_err(ab, "UserPD spawn wait timed out\n");
		ret = -ETIMEDOUT;
		goto err_fw2;
	}

	time_left = wait_for_completion_timeout(&ab_ahb->userpd_ready,
						ATH12K_USERPD_READY_TIMEOUT);
	if (!time_left) {
		ath12k_err(ab, "UserPD ready wait timed out\n");
		ret = -ETIMEDOUT;
		goto err_fw2;
	}

	qcom_smem_state_update_bits(ab_ahb->spawn_state, BIT(ab_ahb->spawn_bit), 0);

	ath12k_dbg(ab, ATH12K_DBG_AHB, "UserPD%d is now UP\n", ab_ahb->userpd_id);

err_fw2:
	release_firmware(fw2);
err_fw:
	release_firmware(fw);
	return ret;
}

static void ath12k_ahb_power_down(struct ath12k_base *ab, bool is_suspend)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	unsigned long time_left;
	u32 pasid;
	int ret;

	qcom_smem_state_update_bits(ab_ahb->stop_state, BIT(ab_ahb->stop_bit),
				    BIT(ab_ahb->stop_bit));

	time_left = wait_for_completion_timeout(&ab_ahb->userpd_stopped,
						ATH12K_USERPD_STOP_TIMEOUT);
	if (!time_left) {
		ath12k_err(ab, "UserPD stop wait timed out\n");
		return;
	}

	qcom_smem_state_update_bits(ab_ahb->stop_state, BIT(ab_ahb->stop_bit), 0);

	if (ab_ahb->scm_auth_enabled) {
		pasid = (u32_encode_bits(ab_ahb->userpd_id, ATH12K_USERPD_ID_MASK)) |
			 ATH12K_AHB_UPD_SWID;
		/* Release the firmware */
		ret = qcom_pas_shutdown(pasid);
		if (ret)
			ath12k_err(ab, "PAS shutdown failed for userPD%d: %d\n",
				   ab_ahb->userpd_id, ret);
	}
}

static void ath12k_ahb_init_qmi_ce_config(struct ath12k_base *ab)
{
	struct ath12k_qmi_ce_cfg *cfg = &ab->qmi.ce_cfg;

	cfg->tgt_ce_len = ab->hw_params->target_ce_count;
	cfg->tgt_ce = ab->hw_params->target_ce_config;
	cfg->svc_to_ce_map_len = ab->hw_params->svc_to_ce_map_len;
	cfg->svc_to_ce_map = ab->hw_params->svc_to_ce_map;
	ab->qmi.service_ins_id = ab->hw_params->qmi_service_ins_id;
}

static void ath12k_ahb_ce_workqueue(struct work_struct *work)
{
	struct ath12k_ce_pipe *ce_pipe = from_work(ce_pipe, work, intr_wq);

	ath12k_ce_per_engine_service(ce_pipe->ab, ce_pipe->pipe_num);

	ath12k_ahb_ce_irq_enable(ce_pipe->ab, ce_pipe->pipe_num);
}

static irqreturn_t ath12k_ahb_ce_interrupt_handler(int irq, void *arg)
{
	struct ath12k_ce_pipe *ce_pipe = arg;

	/* last interrupt received for this CE */
	ce_pipe->timestamp = jiffies;

	ath12k_ahb_ce_irq_disable(ce_pipe->ab, ce_pipe->pipe_num);

	queue_work(system_bh_wq, &ce_pipe->intr_wq);

	return IRQ_HANDLED;
}

static int ath12k_ahb_ext_grp_napi_poll(struct napi_struct *napi, int budget)
{
	struct ath12k_ext_irq_grp *irq_grp = container_of(napi,
						struct ath12k_ext_irq_grp,
						napi);
	struct ath12k_base *ab = irq_grp->ab;
	struct ath12k_dp *dp = ath12k_ab_to_dp(ab);
	int work_done;

	work_done = ath12k_dp_service_srng(dp, irq_grp, budget);
	if (work_done < budget) {
		napi_complete_done(napi, work_done);
		ath12k_ahb_ext_grp_enable(irq_grp);
	}

	if (work_done > budget)
		work_done = budget;

	return work_done;
}

static irqreturn_t ath12k_ahb_ext_interrupt_handler(int irq, void *arg)
{
	struct ath12k_ext_irq_grp *irq_grp = arg;

	/* last interrupt received for this group */
	irq_grp->timestamp = jiffies;

	ath12k_ahb_ext_grp_disable(irq_grp);

	napi_schedule(&irq_grp->napi);

	return IRQ_HANDLED;
}

static int ath12k_ahb_config_ext_irq(struct ath12k_base *ab)
{
	const struct ath12k_hw_ring_mask *ring_mask;
	struct ath12k_ext_irq_grp *irq_grp;
	int i, j, irq, irq_idx, ret;
	u32 num_irq;

	ring_mask = ab->hw_params->ring_mask;
	for (i = 0; i < ATH12K_EXT_IRQ_GRP_NUM_MAX; i++) {
		irq_grp = &ab->ext_irq_grp[i];
		num_irq = 0;

		irq_grp->ab = ab;
		irq_grp->grp_id = i;

		irq_grp->napi_ndev = alloc_netdev_dummy(0);
		if (!irq_grp->napi_ndev)
			return -ENOMEM;

		netif_napi_add(irq_grp->napi_ndev, &irq_grp->napi,
			       ath12k_ahb_ext_grp_napi_poll);

		for (j = 0; j < DP_TCL_NUM_RING_MAX; j++) {
			if (ring_mask->tx[i] &
			    BIT(ab->hal.tcl_to_wbm_rbm_map[j].wbm_ring_num) &&
			    num_irq < ATH12K_EXT_IRQ_NUM_MAX) {
				irq_grp->irqs[num_irq++] =
					wbm2host_tx_completions_ring1 - j;
			}
		}

		for (j = 0; j < ATH12K_EXT_IRQ_NUM_MAX; j++) {
			if (ring_mask->rx[i] & BIT(j) &&
			    num_irq < ATH12K_EXT_IRQ_NUM_MAX) {
				irq_grp->irqs[num_irq++] =
					reo2host_destination_ring1 - j;
			}

			if (ring_mask->rx_err[i] & BIT(j) &&
			    num_irq < ATH12K_EXT_IRQ_NUM_MAX)
				irq_grp->irqs[num_irq++] = reo2host_exception;

			if (ring_mask->rx_wbm_rel[i] & BIT(j) &&
			    num_irq < ATH12K_EXT_IRQ_NUM_MAX)
				irq_grp->irqs[num_irq++] = wbm2host_rx_release;

			if (ring_mask->reo_status[i] & BIT(j) &&
			    num_irq < ATH12K_EXT_IRQ_NUM_MAX)
				irq_grp->irqs[num_irq++] = reo2host_status;

			if (ring_mask->rx_mon_dest[i] & BIT(j) &&
			    num_irq < ATH12K_EXT_IRQ_NUM_MAX)
				irq_grp->irqs[num_irq++] =
					rxdma2host_monitor_destination_mac1;
		}

		irq_grp->num_irq = num_irq;

		for (j = 0; j < irq_grp->num_irq; j++) {
			irq_idx = irq_grp->irqs[j];

			irq = platform_get_irq_byname(ab->pdev,
						      irq_name[irq_idx]);
			ab->irq_num[irq_idx] = irq;
			irq_set_status_flags(irq, IRQ_NOAUTOEN | IRQ_DISABLE_UNLAZY);
			ret = devm_request_irq(ab->dev, irq,
					       ath12k_ahb_ext_interrupt_handler,
					       IRQF_TRIGGER_RISING,
					       irq_name[irq_idx], irq_grp);
			if (ret)
				ath12k_warn(ab, "failed request_irq for %d\n", irq);
		}
	}

	return 0;
}

static int ath12k_ahb_config_irq(struct ath12k_base *ab)
{
	int irq, irq_idx, i;
	int ret;

	/* Configure CE irqs */
	for (i = 0; i < ab->hw_params->ce_count; i++) {
		struct ath12k_ce_pipe *ce_pipe = &ab->ce.ce_pipe[i];

		if (ath12k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR)
			continue;

		irq_idx = ATH12K_IRQ_CE0_OFFSET + i;

		INIT_WORK(&ce_pipe->intr_wq, ath12k_ahb_ce_workqueue);
		irq = platform_get_irq_byname(ab->pdev, irq_name[irq_idx]);
		ret = devm_request_irq(ab->dev, irq, ath12k_ahb_ce_interrupt_handler,
				       IRQF_TRIGGER_RISING, irq_name[irq_idx],
				       ce_pipe);
		if (ret)
			return ret;

		ab->irq_num[irq_idx] = irq;
	}

	/* Configure external interrupts */
	ret = ath12k_ahb_config_ext_irq(ab);

	return ret;
}

static int ath12k_ahb_map_service_to_pipe(struct ath12k_base *ab, u16 service_id,
					  u8 *ul_pipe, u8 *dl_pipe)
{
	const struct service_to_pipe *entry;
	bool ul_set = false, dl_set = false;
	u32 pipedir;
	int i;

	for (i = 0; i < ab->hw_params->svc_to_ce_map_len; i++) {
		entry = &ab->hw_params->svc_to_ce_map[i];

		if (__le32_to_cpu(entry->service_id) != service_id)
			continue;

		pipedir = __le32_to_cpu(entry->pipedir);
		if (pipedir == PIPEDIR_IN || pipedir == PIPEDIR_INOUT) {
			WARN_ON(dl_set);
			*dl_pipe = __le32_to_cpu(entry->pipenum);
			dl_set = true;
		}

		if (pipedir == PIPEDIR_OUT || pipedir == PIPEDIR_INOUT) {
			WARN_ON(ul_set);
			*ul_pipe = __le32_to_cpu(entry->pipenum);
			ul_set = true;
		}
	}

	if (WARN_ON(!ul_set || !dl_set))
		return -ENOENT;

	return 0;
}

const struct ath12k_hif_ops ath12k_ahb_hif_ops = {
	.start = ath12k_ahb_start,
	.stop = ath12k_ahb_stop,
	.read32 = ath12k_ahb_read32,
	.write32 = ath12k_ahb_write32,
	.irq_enable = ath12k_ahb_ext_irq_enable,
	.irq_disable = ath12k_ahb_ext_irq_disable,
	.map_service_to_pipe = ath12k_ahb_map_service_to_pipe,
	.power_up = ath12k_ahb_power_up,
	.power_down = ath12k_ahb_power_down,
};
EXPORT_SYMBOL(ath12k_ahb_hif_ops);

/* WCN7750: WLAN Q6 on the WPSS remoteproc, MSI as GIC SPIs, register BAR
 * reported over QMI. Reuses the PCI CE/DP IRQ and window helpers.
 *
 * Rails from the reference qcom,vdd-*-config; shared with Bluetooth.
 */
static const struct {
	const char *name;
	int min_uv;
	int max_uv;
} ath12k_ahb_wcn7750_vreg[] = {
	{ "vdd-cx-mx",	 880000, 1040000 },
	{ "vdd-1.8-xo",	1856000, 2040000 },
	{ "vdd-1.3-rfa", 1256000, 1408000 },
	{ "vdd-1.8-io",	1800000, 1800000 },
};

/* host_cap ddr_range: the IOVA hole between the iommu-addresses
 * reservations, and the MSA carveout.
 */
static void ath12k_ahb_wcn_parse_host_ddr_ranges(struct ath12k_base *ab)
{
	struct device_node *np = ab->dev->of_node;
	struct of_phandle_iterator it;
	struct resource res;
	int err;

	if (!of_reserved_mem_region_to_resource_byname(np, "msa", &res)) {
		ab->host_ddr_msa_start = res.start;
		ab->host_ddr_msa_size = resource_size(&res);
	} else {
		ath12k_warn(ab, "no \"msa\" memory-region, firmware will not be told about MSA\n");
	}

	of_for_each_phandle(&it, err, np, "memory-region", NULL, 0) {
		phys_addr_t resv_start[2] = {};
		const __be32 *maps, *end;
		size_t resv_len[2] = {};
		int nr = 0, size;

		maps = of_get_property(it.node, "iommu-addresses", &size);
		if (!maps)
			continue;

		end = maps + size / sizeof(__be32);
		while (maps < end && nr < 2) {
			struct device_node *target;

			target = of_find_node_by_phandle(be32_to_cpup(maps++));
			of_node_put(target);
			if (target != np)
				break;

			maps = of_translate_dma_region(np, maps,
						       &resv_start[nr],
						       &resv_len[nr]);
			if (!maps)
				break;
			nr++;
		}

		/* below the window, then above it */
		if (nr != 2 || resv_start[0] != 0 ||
		    resv_start[1] <= resv_start[0] + resv_len[0]) {
			ath12k_warn(ab, "unexpected iommu-addresses layout in %pOF\n",
				    it.node);
			continue;
		}

		ab->host_ddr_iova_start = resv_start[0] + resv_len[0];
		ab->host_ddr_iova_size = resv_start[1] - ab->host_ddr_iova_start;
		of_node_put(it.node);
		return;
	}

	ath12k_warn(ab, "no iommu-addresses reservations, firmware will not be told about the IOVA window\n");
}

static int ath12k_ahb_wcn_get_vregs(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	int i, ret;

	ab_ahb->num_vregs = ARRAY_SIZE(ath12k_ahb_wcn7750_vreg);
	ab_ahb->vregs = devm_kcalloc(ab->dev, ab_ahb->num_vregs,
				     sizeof(*ab_ahb->vregs), GFP_KERNEL);
	if (!ab_ahb->vregs)
		return -ENOMEM;

	for (i = 0; i < ab_ahb->num_vregs; i++)
		ab_ahb->vregs[i].supply = ath12k_ahb_wcn7750_vreg[i].name;

	ret = devm_regulator_bulk_get(ab->dev, ab_ahb->num_vregs, ab_ahb->vregs);
	if (ret)
		return dev_err_probe(ab->dev, ret, "failed to get WLAN supplies\n");

	/* WLAN_EN releases the companion die from reset. */
	ab_ahb->wlan_en_gpio = devm_gpiod_get_optional(ab->dev, "wlan-en",
						       GPIOD_OUT_LOW);
	if (IS_ERR(ab_ahb->wlan_en_gpio))
		return dev_err_probe(ab->dev, PTR_ERR(ab_ahb->wlan_en_gpio),
				     "failed to get wlan-en GPIO\n");

	if (!ab_ahb->wlan_en_gpio)
		ath12k_warn(ab, "no wlan-en GPIO: companion die may stay in reset\n");

	/* PCIe->DDR path the die's BHI fetch and DMA use. */
	ab_ahb->icc_mem = devm_of_icc_get(ab->dev, "pcie-mem");
	if (IS_ERR(ab_ahb->icc_mem))
		return dev_err_probe(ab->dev, PTR_ERR(ab_ahb->icc_mem),
				     "failed to get pcie-mem interconnect\n");

	if (!ab_ahb->icc_mem)
		ath12k_warn(ab, "no pcie-mem interconnect path\n");

	return 0;
}

static int ath12k_ahb_wcn_vregs_on(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	int i, ret;

	for (i = 0; i < ab_ahb->num_vregs; i++) {
		ret = regulator_set_voltage(ab_ahb->vregs[i].consumer,
					    ath12k_ahb_wcn7750_vreg[i].min_uv,
					    ath12k_ahb_wcn7750_vreg[i].max_uv);
		if (ret) {
			ath12k_err(ab, "failed to set %s to %d-%duV: %d\n",
				   ath12k_ahb_wcn7750_vreg[i].name,
				   ath12k_ahb_wcn7750_vreg[i].min_uv,
				   ath12k_ahb_wcn7750_vreg[i].max_uv, ret);
			return ret;
		}
	}

	ret = regulator_bulk_enable(ab_ahb->num_vregs, ab_ahb->vregs);
	if (ret) {
		ath12k_err(ab, "failed to enable WLAN supplies: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ath12k_ahb_wcn_vregs_off(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	regulator_bulk_disable(ab_ahb->num_vregs, ab_ahb->vregs);
}

static const struct ath12k_msi_config ath12k_ahb_msi_config_wcn7750 = {
	.total_vectors = 28,
	.total_users = 2,
	.users = (struct ath12k_msi_user[]) {
		{ .name = "CE", .num_vectors = 10, .base_vector = 0 },
		{ .name = "DP", .num_vectors = 18, .base_vector = 10 },
	},
};

/* Same four-region BAR remap as the PCI parts: 0x80000 dynamic window,
 * 0x100000 static CE window, 0x180000 static UMAC window, window register
 * at 0x3278.
 */
#define ATH12K_AHB_WCN_WINDOW_REG_ADDRESS	0x3278
#define ATH12K_AHB_WCN_WINDOW_ENABLE_BIT	0x40000000
#define ATH12K_AHB_WCN_WINDOW_VALUE_MASK	GENMASK(24, 19)
#define ATH12K_AHB_WCN_WINDOW_STATIC_MASK	GENMASK(31, 6)

/* Re-armed on every bring-up: the target resets the window register on restart. */
static void ath12k_ahb_wcn_cfg_static_window(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 umac_window, ce_window, window;

	if (!ab->mem)
		return;

	umac_window = u32_get_bits(ab->reg_base->umac_base,
				   ATH12K_AHB_WCN_WINDOW_VALUE_MASK);
	ce_window = u32_get_bits(ab->reg_base->ce_reg_base,
				 ATH12K_AHB_WCN_WINDOW_VALUE_MASK);
	window = (umac_window << 12) | (ce_window << 6);

	/* the datapath may be selecting dynamic windows concurrently */
	spin_lock_bh(&ab_ahb->window_lock);
	ab_ahb->register_window = window;

	iowrite32(ATH12K_AHB_WCN_WINDOW_ENABLE_BIT | window,
		  ab->mem + ATH12K_AHB_WCN_WINDOW_REG_ADDRESS);
	ioread32(ab->mem + ATH12K_AHB_WCN_WINDOW_REG_ADDRESS);
	spin_unlock_bh(&ab_ahb->window_lock);

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "static register windows set to 0x%08x\n",
		   window);
}

static void ath12k_ahb_wcn_select_window(struct ath12k_base *ab, u32 offset)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 window = u32_get_bits(offset, ATH12K_AHB_WCN_WINDOW_VALUE_MASK);

	lockdep_assert_held(&ab_ahb->window_lock);

	/* Keep the static window configuration, change only the dynamic one. */
	window |= ab_ahb->register_window & ATH12K_AHB_WCN_WINDOW_STATIC_MASK;

	if (window != ab_ahb->register_window) {
		iowrite32(ATH12K_AHB_WCN_WINDOW_ENABLE_BIT | window,
			  ab->mem + ATH12K_AHB_WCN_WINDOW_REG_ADDRESS);
		ioread32(ab->mem + ATH12K_AHB_WCN_WINDOW_REG_ADDRESS);
		ab_ahb->register_window = window;
	}
}

static u32 ath12k_ahb_wcn_get_window_start(struct ath12k_base *ab, u32 offset)
{
	if ((offset ^ ab->reg_base->umac_base) < ATH12K_PCI_WINDOW_RANGE_MASK)
		return 3 * ATH12K_PCI_WINDOW_START;

	if ((offset ^ ab->reg_base->ce_reg_base) < ATH12K_PCI_WINDOW_RANGE_MASK)
		return 2 * ATH12K_PCI_WINDOW_START;

	return ATH12K_PCI_WINDOW_START;
}

static u32 ath12k_ahb_wcn_read32(struct ath12k_base *ab, u32 offset)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 window_start, val;

	if (offset < ATH12K_PCI_WINDOW_START)
		return ioread32(ab->mem + offset);

	window_start = ath12k_ahb_wcn_get_window_start(ab, offset);

	if (window_start != ATH12K_PCI_WINDOW_START)
		return ioread32(ab->mem + window_start +
				(offset & ATH12K_PCI_WINDOW_RANGE_MASK));

	spin_lock_bh(&ab_ahb->window_lock);
	ath12k_ahb_wcn_select_window(ab, offset);
	val = ioread32(ab->mem + window_start +
		       (offset & ATH12K_PCI_WINDOW_RANGE_MASK));
	spin_unlock_bh(&ab_ahb->window_lock);

	return val;
}

static void ath12k_ahb_wcn_write32(struct ath12k_base *ab, u32 offset, u32 value)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 window_start;

	if (offset < ATH12K_PCI_WINDOW_START) {
		iowrite32(value, ab->mem + offset);
		return;
	}

	window_start = ath12k_ahb_wcn_get_window_start(ab, offset);

	if (window_start != ATH12K_PCI_WINDOW_START) {
		iowrite32(value, ab->mem + window_start +
			  (offset & ATH12K_PCI_WINDOW_RANGE_MASK));
		return;
	}

	spin_lock_bh(&ab_ahb->window_lock);
	ath12k_ahb_wcn_select_window(ab, offset);
	iowrite32(value, ab->mem + window_start +
		  (offset & ATH12K_PCI_WINDOW_RANGE_MASK));
	spin_unlock_bh(&ab_ahb->window_lock);
}

static void ath12k_ahb_wcn_get_msi_address(struct ath12k_base *ab,
					   u32 *msi_addr_lo, u32 *msi_addr_hi)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	*msi_addr_lo = ab_ahb->msi_addr_lo;
	*msi_addr_hi = ab_ahb->msi_addr_hi;
}

/* icnss2 layout: sequence number in the upper half, message id below */
static void ath12k_ahb_wcn_send_smp2p(struct ath12k_base *ab,
				      struct qcom_smem_state *state,
				      enum ath12k_ahb_smp2p_msg_id msg_id)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 value;
	int ret;

	if (!state)
		return;

	value = u32_encode_bits(ab_ahb->smp2p_seq++, ATH12K_AHB_SMP2P_SMEM_SEQ_NO) |
		u32_encode_bits(msg_id, ATH12K_AHB_SMP2P_SMEM_MSG);

	ret = qcom_smem_state_update_bits(state,
					  ATH12K_AHB_SMP2P_SMEM_VALUE_MASK,
					  value);
	if (ret)
		ath12k_warn(ab, "failed to send smp2p msg %u: %d\n", msg_id, ret);
	else
		ath12k_dbg(ab, ATH12K_DBG_AHB, "smp2p msg %u value 0x%08x\n",
			   msg_id, value);
}

#define ATH12K_AHB_WCN_CE_IRQ_MAX_DEPTH	8

/* Unmask from the real irq state; the flag can be stale after a restart. */
static void ath12k_ahb_wcn_ce_irqs_force_enable(struct ath12k_base *ab)
{
	struct irq_data *d;
	int i, irq, depth;

	for (i = 0; i < ab->hw_params->ce_count; i++) {
		if (ath12k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR)
			continue;

		irq = ab->irq_num[ATH12K_PCI_IRQ_CE0_OFFSET + i];
		d = irq > 0 ? irq_get_irq_data(irq) : NULL;
		if (!d)
			continue;

		for (depth = 0; irqd_irq_disabled(d); depth++) {
			if (depth >= ATH12K_AHB_WCN_CE_IRQ_MAX_DEPTH) {
				ath12k_warn(ab, "ce%d irq %d still masked after %d enables\n",
					    i, irq, depth);
				break;
			}
			enable_irq(irq);
		}
	}

	set_bit(ATH12K_FLAG_CE_IRQ_ENABLED, &ab->dev_flags);
}

/* The only message the firmware accepts while down. */
static void ath12k_ahb_wcn_reset_smp2p(struct ath12k_base *ab,
				       struct qcom_smem_state *state)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	u32 value;
	int ret;

	if (!state)
		return;

	ab_ahb->smp2p_seq = 0;

	value = u32_encode_bits(ab_ahb->smp2p_seq, ATH12K_AHB_SMP2P_SMEM_SEQ_NO) |
		u32_encode_bits(ATH12K_AHB_RESET_MSG, ATH12K_AHB_SMP2P_SMEM_MSG);

	ret = qcom_smem_state_update_bits(state,
					  ATH12K_AHB_SMP2P_SMEM_VALUE_MASK,
					  value);
	if (ret)
		ath12k_warn(ab, "failed to reset smp2p entry: %d\n", ret);
	else
		ath12k_dbg(ab, ATH12K_DBG_AHB, "smp2p entry reset\n");
}

static int ath12k_ahb_wcn_start(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	ath12k_ahb_wcn_send_smp2p(ab, ab_ahb->ep_power_save_state,
				  ATH12K_AHB_PCI_EP_POWER_SAVE_EXIT);

	ath12k_ahb_wcn_ce_irqs_force_enable(ab);

	ath12k_ce_rx_post_buf(ab);

	return 0;
}

static void ath12k_ahb_wcn_stop(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	/* or the next recovery trips "Unbalanced enable for IRQ" */
	ath12k_pci_hif_ce_irq_disable(ab);

	if (test_bit(ATH12K_FLAG_CRASH_FLUSH, &ab->dev_flags)) {
		ath12k_ahb_wcn_reset_smp2p(ab, ab_ahb->ep_power_save_state);
	} else {
		ath12k_ahb_wcn_send_smp2p(ab, ab_ahb->ep_power_save_state,
					  ATH12K_AHB_PCI_EP_POWER_SAVE_ENTER);
	}

	ath12k_ce_cleanup_pipes(ab);
}

/* rail/XO settling before the die leaves reset */
#define ATH12K_AHB_WCN_DIE_SETTLE_US	4000

/* hold time for a real power cycle */
#define ATH12K_AHB_WCN_DIE_RESET_MS	20

/* rproc subdev prepare(): rails, PCIe->DDR vote, then release from reset. */
static int ath12k_ahb_wcn_die_power_on(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	int ret;

	ret = ath12k_ahb_wcn_vregs_on(ab);
	if (ret)
		return ret;

	if (ab_ahb->icc_mem) {
		ret = icc_set_bw(ab_ahb->icc_mem, 0, MBps_to_icc(800));
		if (ret) {
			ath12k_err(ab, "failed to vote pcie-mem bandwidth: %d\n",
				   ret);
			ath12k_ahb_wcn_vregs_off(ab);
			return ret;
		}
	}

	if (ab_ahb->wlan_en_gpio) {
		gpiod_set_value_cansleep(ab_ahb->wlan_en_gpio, 1);
		usleep_range(ATH12K_AHB_WCN_DIE_SETTLE_US,
			     ATH12K_AHB_WCN_DIE_SETTLE_US + 1000);
	}

	return 0;
}

static void ath12k_ahb_wcn_die_power_off(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	if (ab_ahb->wlan_en_gpio)
		gpiod_set_value_cansleep(ab_ahb->wlan_en_gpio, 0);

	if (ab_ahb->icc_mem)
		icc_set_bw(ab_ahb->icc_mem, 0, 0);

	ath12k_ahb_wcn_vregs_off(ab);

	/* remoteproc recovery calls unprepare()/prepare() ~1 ms apart */
	msleep(ATH12K_AHB_WCN_DIE_RESET_MS);

	/* the window register does not survive the power cycle */
	spin_lock_bh(&ab_ahb->window_lock);
	ab_ahb->register_window = 0;
	spin_unlock_bh(&ab_ahb->window_lock);
}

static struct ath12k_base *ath12k_ahb_wcn_subdev_to_ab(struct rproc_subdev *subdev)
{
	struct ath12k_ahb *ab_ahb = container_of(subdev, struct ath12k_ahb,
						 wpss_subdev);

	return ab_ahb->ab;
}

static int ath12k_ahb_wcn_subdev_prepare(struct rproc_subdev *subdev)
{
	return ath12k_ahb_wcn_die_power_on(ath12k_ahb_wcn_subdev_to_ab(subdev));
}

/* Only notification that the firmware died; the die is still powered here. */
static void ath12k_ahb_wcn_subdev_stop(struct rproc_subdev *subdev, bool crashed)
{
	struct ath12k_base *ab = ath12k_ahb_wcn_subdev_to_ab(subdev);

	if (!crashed)
		return;

	ath12k_core_firmware_crashed(ab);
}

static void ath12k_ahb_wcn_subdev_unprepare(struct rproc_subdev *subdev)
{
	ath12k_ahb_wcn_die_power_off(ath12k_ahb_wcn_subdev_to_ab(subdev));
}

/* Subdev of the WPSS rproc, so remoteproc's own crash restart brackets the
 * die as well.
 */
static void ath12k_ahb_wcn_add_subdev(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	if (ab_ahb->wpss_subdev_added)
		return;

	ab_ahb->wpss_subdev.prepare = ath12k_ahb_wcn_subdev_prepare;
	ab_ahb->wpss_subdev.stop = ath12k_ahb_wcn_subdev_stop;
	ab_ahb->wpss_subdev.unprepare = ath12k_ahb_wcn_subdev_unprepare;

	rproc_add_subdev(ab_ahb->wpss_rproc, &ab_ahb->wpss_subdev);
	ab_ahb->wpss_subdev_added = true;
}

static void ath12k_ahb_wcn_remove_subdev(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	if (!ab_ahb->wpss_subdev_added)
		return;

	rproc_remove_subdev(ab_ahb->wpss_rproc, &ab_ahb->wpss_subdev);
	ab_ahb->wpss_subdev_added = false;
}

static int ath12k_ahb_wcn_power_up(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	int ret;

	ret = rproc_boot(ab_ahb->wpss_rproc);
	if (ret)
		ath12k_err(ab, "failed to boot WPSS Q6: %d\n", ret);

	return ret;
}

static void ath12k_ahb_wcn_power_down(struct ath12k_base *ab, bool is_suspend)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	rproc_shutdown(ab_ahb->wpss_rproc);
}

const struct ath12k_hif_ops ath12k_ahb_hif_ops_wcn7750 = {
	.start = ath12k_ahb_wcn_start,
	.stop = ath12k_ahb_wcn_stop,
	.read32 = ath12k_ahb_wcn_read32,
	.write32 = ath12k_ahb_wcn_write32,
	.power_up = ath12k_ahb_wcn_power_up,
	.power_down = ath12k_ahb_wcn_power_down,
	.irq_enable = ath12k_pci_ext_irq_enable,
	.irq_disable = ath12k_pci_ext_irq_disable,
	.get_msi_address = ath12k_ahb_wcn_get_msi_address,
	.get_user_msi_vector = ath12k_pci_get_user_msi_assignment,
	.map_service_to_pipe = ath12k_pci_map_service_to_pipe,
	.ce_irq_enable = ath12k_pci_hif_ce_irq_enable,
	.ce_irq_disable = ath12k_pci_hif_ce_irq_disable,
	.get_ce_msi_idx = ath12k_pci_get_ce_msi_idx,
	.cfg_static_window = ath12k_ahb_wcn_cfg_static_window,
};
EXPORT_SYMBOL(ath12k_ahb_hif_ops_wcn7750);

/* icnss_aop_pdc_reconfig(): the die's autonomous rail requests need these. */
static int ath12k_ahb_wcn_pdc_init(struct ath12k_base *ab)
{
	struct qmp *qmp;
	struct device_node *node = ab->dev->of_node;
	const char *msg;
	int count, i, ret;

	count = of_property_count_strings(node, "qcom,pdc-init-table");
	if (count <= 0) {
		ath12k_dbg(ab, ATH12K_DBG_AHB, "no pdc init table\n");
		return 0;
	}

	qmp = qmp_get(ab->dev);
	if (IS_ERR(qmp))
		return PTR_ERR(qmp);

	for (i = 0; i < count; i++) {
		ret = of_property_read_string_index(node, "qcom,pdc-init-table",
						    i, &msg);
		if (ret)
			break;

		ret = qmp_send(qmp, "%s", msg);
		if (ret) {
			ath12k_warn(ab, "failed to send pdc msg '%s': %d\n",
				    msg, ret);
			break;
		}

		ath12k_dbg(ab, ATH12K_DBG_AHB, "pdc msg '%s' sent\n", msg);
	}

	ath12k_dbg(ab, ATH12K_DBG_AHB, "pdc init table sent (%d entries)\n", i);

	qmp_put(qmp);

	return 0;
}

/* gates endpoint power save on the internal PCIe link */
static int ath12k_ahb_wcn_get_ep_power_save_state(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct qcom_smem_state *state;
	u32 bit;

	state = devm_qcom_smem_state_get(ab->dev, "wlan-ep-powersave-smp2p-out",
					 &bit);
	if (IS_ERR(state))
		return PTR_ERR(state);

	ab_ahb->ep_power_save_state = state;

	return 0;
}

/* MSI data base = first SPI number + this offset (ath11k/icnss) */
#define ATH12K_AHB_WCN_MSI_SPI_OFFSET	32

static void ath12k_ahb_wcn_unmap_msi_addr(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	if (!ab_ahb->msi_addr_iova)
		return;

	dma_unmap_resource(ab->dev, ab_ahb->msi_addr_iova, PAGE_SIZE,
			   DMA_FROM_DEVICE, 0);
	ab_ahb->msi_addr_iova = 0;
}

static int ath12k_ahb_wcn_setup_msi(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct platform_device *pdev = ab->pdev;
	dma_addr_t msi_addr_iova;
	phys_addr_t msi_addr_pa;
	struct resource *res;
	u32 int_prop;
	int ret;

	ab->msi_config = &ath12k_ahb_msi_config_wcn7750;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "msi_addr");
	if (!res) {
		ath12k_err(ab, "failed to fetch msi_addr resource\n");
		return -ENOENT;
	}

	msi_addr_pa = res->start;
	msi_addr_iova = dma_map_resource(ab->dev, msi_addr_pa, PAGE_SIZE,
					 DMA_FROM_DEVICE, 0);
	if (dma_mapping_error(ab->dev, msi_addr_iova))
		return -ENOMEM;

	ab_ahb->msi_addr_iova = msi_addr_iova;
	ab_ahb->msi_addr_lo = lower_32_bits(msi_addr_iova);
	ab_ahb->msi_addr_hi = upper_32_bits(msi_addr_iova);

	/* 2nd cell of the first interrupt = first SPI number */
	ret = of_property_read_u32_index(ab->dev->of_node, "interrupts", 1,
					 &int_prop);
	if (ret) {
		ath12k_ahb_wcn_unmap_msi_addr(ab);
		return ret;
	}

	ab->msi_ep_base_data = int_prop + ATH12K_AHB_WCN_MSI_SPI_OFFSET;
	ab->multi_msi_vectors = true;
	ab->msi_irq_flags = IRQF_SHARED;

	return 0;
}

static int ath12k_ahb_wcn_get_rproc(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct device *dev = ab->dev;
	struct rproc *prproc;
	phandle rproc_phandle;

	if (of_property_read_u32(dev->of_node, "qcom,rproc", &rproc_phandle)) {
		ath12k_err(ab, "failed to get qcom,rproc handle\n");
		return -ENOENT;
	}

	prproc = rproc_get_by_phandle(rproc_phandle);
	if (!prproc)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "failed to get WPSS rproc\n");

	ab_ahb->wpss_rproc = prproc;

	return 0;
}

static irqreturn_t ath12k_userpd_irq_handler(int irq, void *data)
{
	struct ath12k_base *ab = data;
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	if (irq == ab_ahb->userpd_irq_num[ATH12K_USERPD_SPAWN_IRQ]) {
		complete(&ab_ahb->userpd_spawned);
	} else if (irq == ab_ahb->userpd_irq_num[ATH12K_USERPD_READY_IRQ]) {
		complete(&ab_ahb->userpd_ready);
	} else if (irq == ab_ahb->userpd_irq_num[ATH12K_USERPD_STOP_ACK_IRQ])	{
		complete(&ab_ahb->userpd_stopped);
	} else {
		ath12k_err(ab, "Invalid userpd interrupt\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int ath12k_ahb_config_rproc_irq(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	int i, ret;
	char *upd_irq_name;

	for (i = 0; i < ATH12K_USERPD_MAX_IRQ; i++) {
		ab_ahb->userpd_irq_num[i] = platform_get_irq_byname(ab->pdev,
								    ath12k_userpd_irq[i]);
		if (ab_ahb->userpd_irq_num[i] < 0)
			return ab_ahb->userpd_irq_num[i];

		upd_irq_name = devm_kzalloc(&ab->pdev->dev, ATH12K_UPD_IRQ_WRD_LEN,
					    GFP_KERNEL);
		if (!upd_irq_name)
			return -ENOMEM;

		scnprintf(upd_irq_name, ATH12K_UPD_IRQ_WRD_LEN, "UserPD%u-%s",
			  ab_ahb->userpd_id, ath12k_userpd_irq[i]);
		ret = devm_request_threaded_irq(&ab->pdev->dev, ab_ahb->userpd_irq_num[i],
						NULL, ath12k_userpd_irq_handler,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						upd_irq_name, ab);
		if (ret)
			return dev_err_probe(&ab->pdev->dev, ret,
					     "Request %s irq failed: %d\n",
					     ath12k_userpd_irq[i], ret);
	}

	ab_ahb->spawn_state = devm_qcom_smem_state_get(&ab->pdev->dev, "spawn",
						       &ab_ahb->spawn_bit);
	if (IS_ERR(ab_ahb->spawn_state))
		return dev_err_probe(&ab->pdev->dev, PTR_ERR(ab_ahb->spawn_state),
				     "Failed to acquire spawn state\n");

	ab_ahb->stop_state = devm_qcom_smem_state_get(&ab->pdev->dev, "stop",
						      &ab_ahb->stop_bit);
	if (IS_ERR(ab_ahb->stop_state))
		return dev_err_probe(&ab->pdev->dev, PTR_ERR(ab_ahb->stop_state),
				     "Failed to acquire stop state\n");

	init_completion(&ab_ahb->userpd_spawned);
	init_completion(&ab_ahb->userpd_ready);
	init_completion(&ab_ahb->userpd_stopped);
	return 0;
}

static int ath12k_ahb_root_pd_state_notifier(struct notifier_block *nb,
					     const unsigned long event, void *data)
{
	struct ath12k_ahb_rproc_info *rproc_info =
		container_of(nb, struct ath12k_ahb_rproc_info, root_pd_nb);

	if (event == ATH12K_RPROC_AFTER_POWERUP) {
		ath12k_generic_dbg(ATH12K_DBG_AHB, "Root PD is UP\n");
		complete(&rproc_info->rootpd_ready);
	}

	return 0;
}

static int ath12k_ahb_register_rproc_notifier(void)
{
	int ret;

	lockdep_assert_held(&ath12k_rproc_info_lock);

	if (g_rproc_info->root_pd_notifier)
		return 0;

	g_rproc_info->root_pd_nb.notifier_call = ath12k_ahb_root_pd_state_notifier;

	g_rproc_info->root_pd_notifier =
			qcom_register_ssr_notifier(g_rproc_info->tgt_rproc->name,
						   &g_rproc_info->root_pd_nb);
	if (IS_ERR(g_rproc_info->root_pd_notifier)) {
		ret = PTR_ERR(g_rproc_info->root_pd_notifier);
		g_rproc_info->root_pd_notifier = NULL;
		return ret;
	}

	return 0;
}

static void ath12k_ahb_unregister_rproc_notifier(void)
{
	lockdep_assert_held(&ath12k_rproc_info_lock);

	if (!g_rproc_info->root_pd_notifier)
		return;

	qcom_unregister_ssr_notifier(g_rproc_info->root_pd_notifier,
				     &g_rproc_info->root_pd_nb);
	g_rproc_info->root_pd_notifier = NULL;
}

static void ath12k_ahb_cleanup_userpd(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct ath12k_ahb_rproc_info *rproc_info = ab_ahb->rproc_info;

	lockdep_assert_held(&ath12k_rproc_info_lock);

	if (!rproc_info)
		return;

	rproc_info->userpd[ab_ahb->userpd_id - 1] = NULL;
	rproc_info->num_userpd--;
	ab_ahb->rproc_info = NULL;
}

static struct ath12k_ahb_rproc_info *ath12k_ahb_rproc_info_alloc(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct ath12k_ahb_rproc_info *rproc_info;

	lockdep_assert_held(&ath12k_rproc_info_lock);

	rproc_info = kzalloc_obj(*rproc_info);
	if (!rproc_info)
		return NULL;

	rproc_info->rootpd_booted_by_driver = false;
	rproc_info->userpd[ab_ahb->userpd_id - 1] = ab_ahb;
	rproc_info->num_userpd = 1;
	init_completion(&rproc_info->rootpd_ready);
	ab_ahb->rproc_info = rproc_info;

	return rproc_info;
}

static int ath12k_ahb_get_rproc(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct device *dev = ab->dev;
	struct device_node *np;
	struct rproc *prproc;
	int ret;

	lockdep_assert_held(&ath12k_rproc_info_lock);

	if (ab_ahb->userpd_id > ATH12K_MAX_DEVICES)
		return -ENOSPC;

	if (g_rproc_info) {
		if (g_rproc_info->num_userpd >= ATH12K_MAX_DEVICES) {
			ath12k_err(ab, "Max UserPD limit reached\n");
			return -ENOSPC;
		}

		g_rproc_info->userpd[ab_ahb->userpd_id - 1] = ab_ahb;
		g_rproc_info->num_userpd++;
		ab_ahb->rproc_info = g_rproc_info;
		return 0;
	}

	g_rproc_info = ath12k_ahb_rproc_info_alloc(ab);
	if (!g_rproc_info)
		return -ENOMEM;

	np = of_parse_phandle(dev->of_node, "qcom,rproc", 0);
	if (!np) {
		ath12k_err(ab, "failed to get q6_rproc handle\n");
		ret = -ENOENT;
		goto err_free_rproc_info;
	}

	prproc = rproc_get_by_phandle(np->phandle);
	of_node_put(np);
	if (!prproc) {
		ret = dev_err_probe(&ab->pdev->dev, -EPROBE_DEFER,
				    "failed to get rproc\n");
		goto err_free_rproc_info;
	}

	g_rproc_info->tgt_rproc = prproc;
	return 0;

err_free_rproc_info:
	ab_ahb->rproc_info = NULL;
	kfree(g_rproc_info);
	g_rproc_info = NULL;
	return ret;
}

static int ath12k_ahb_boot_root_pd(struct ath12k_base *ab)
{
	unsigned long time_left;
	int ret;

	lockdep_assert_held(&ath12k_rproc_info_lock);
	reinit_completion(&g_rproc_info->rootpd_ready);

	ret = rproc_boot(g_rproc_info->tgt_rproc);
	if (ret < 0) {
		ath12k_err(ab, "RootPD boot failed\n");
		return ret;
	}

	time_left = wait_for_completion_timeout(&g_rproc_info->rootpd_ready,
						ATH12K_ROOTPD_READY_TIMEOUT);
	if (!time_left) {
		ath12k_err(ab, "RootPD ready wait timed out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ath12k_ahb_configure_rproc(struct ath12k_base *ab)
{
	int ret;

	mutex_lock(&ath12k_rproc_info_lock);

	ret = ath12k_ahb_get_rproc(ab);
	if (ret < 0) {
		mutex_unlock(&ath12k_rproc_info_lock);
		return ret;
	}

	ret = ath12k_ahb_register_rproc_notifier();
	if (ret < 0) {
		ret = dev_err_probe(&ab->pdev->dev, ret,
				    "failed to register rproc notifier\n");
		goto err_cleanup_userpd;
	}

	if (g_rproc_info->tgt_rproc->state != RPROC_RUNNING) {
		ret = ath12k_ahb_boot_root_pd(ab);
		if (ret < 0) {
			ath12k_err(ab, "failed to boot the remote processor Q6\n");
			goto err_unreg_notifier;
		}
		g_rproc_info->rootpd_booted_by_driver = true;
	}

	mutex_unlock(&ath12k_rproc_info_lock);
	return 0;

err_unreg_notifier:
	ath12k_ahb_unregister_rproc_notifier();

err_cleanup_userpd:
	ath12k_ahb_cleanup_userpd(ab);

	if (g_rproc_info && !g_rproc_info->num_userpd) {
		rproc_put(g_rproc_info->tgt_rproc);
		kfree(g_rproc_info);
		g_rproc_info = NULL;
	}

	mutex_unlock(&ath12k_rproc_info_lock);
	return ret;
}

static void ath12k_ahb_deconfigure_rproc(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct ath12k_ahb_rproc_info *rproc_info = ab_ahb->rproc_info;

	lockdep_assert_held(&ath12k_rproc_info_lock);

	if (!rproc_info || !g_rproc_info)
		return;

	ath12k_ahb_cleanup_userpd(ab);

	if (!g_rproc_info->num_userpd) {
		ath12k_ahb_unregister_rproc_notifier();

		if (g_rproc_info->rootpd_booted_by_driver &&
		    g_rproc_info->tgt_rproc->state == RPROC_RUNNING)
			rproc_shutdown(g_rproc_info->tgt_rproc);

		rproc_put(g_rproc_info->tgt_rproc);
		kfree(g_rproc_info);
		g_rproc_info = NULL;
	}
}

static int ath12k_ahb_resource_init(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);
	struct platform_device *pdev = ab->pdev;
	struct resource *mem_res;
	int ret;

	/* WCN7750: no reg window until QMI reports it, no xo clock (WPSS PAS owns it). */
	if (ab->hybrid_bus)
		return 0;

	ab->mem = devm_platform_get_and_ioremap_resource(pdev, 0, &mem_res);
	if (IS_ERR(ab->mem)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(ab->mem), "ioremap error\n");
		goto out;
	}

	ab->mem_len = resource_size(mem_res);

	if (ab->hw_params->ce_remap) {
		const struct ce_remap *ce_remap = ab->hw_params->ce_remap;
		/* CE register space is moved out of WCSS and the space is not
		 * contiguous, hence remapping the CE registers to a new space
		 * for accessing them.
		 */
		ab->mem_ce = ioremap(ce_remap->base, ce_remap->size);
		if (!ab->mem_ce) {
			dev_err(&pdev->dev, "ce ioremap error\n");
			ret = -ENOMEM;
			goto err_mem_unmap;
		}
		ab->ce_remap = true;
		ab->cmem_offset = ce_remap->cmem_offset;
		ab->ce_remap_base_addr = ce_remap->base;
	}

	ab_ahb->xo_clk = devm_clk_get(ab->dev, "xo");
	if (IS_ERR(ab_ahb->xo_clk)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(ab_ahb->xo_clk),
				    "failed to get xo clock\n");
		goto err_mem_ce_unmap;
	}

	ret = clk_prepare_enable(ab_ahb->xo_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable gcc_xo_clk: %d\n", ret);
		goto err_clock_deinit;
	}

	return 0;

err_clock_deinit:
	devm_clk_put(ab->dev, ab_ahb->xo_clk);

err_mem_ce_unmap:
	ab_ahb->xo_clk = NULL;
	if (ab->hw_params->ce_remap)
		iounmap(ab->mem_ce);

err_mem_unmap:
	ab->mem_ce = NULL;
	devm_iounmap(ab->dev, ab->mem);

out:
	ab->mem = NULL;
	return ret;
}

static void ath12k_ahb_resource_deinit(struct ath12k_base *ab)
{
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	/* hybrid: ab->mem is a devm_ioremap() of the QMI-reported BAR */
	if (!ab->hybrid_bus && ab->mem)
		devm_iounmap(ab->dev, ab->mem);

	if (ab->mem_ce && ab->ce_remap)
		iounmap(ab->mem_ce);

	ab->mem = NULL;
	ab->mem_ce = NULL;

	if (ab_ahb->xo_clk) {
		clk_disable_unprepare(ab_ahb->xo_clk);
		devm_clk_put(ab->dev, ab_ahb->xo_clk);
		ab_ahb->xo_clk = NULL;
	}
}

static enum ath12k_device_family
ath12k_ahb_get_device_family(const struct platform_device *pdev)
{
	enum ath12k_device_family device_family_id;
	struct ath12k_ahb_driver *driver;
	const struct of_device_id *of_id;

	for (device_family_id = ATH12K_DEVICE_FAMILY_START;
	     device_family_id < ATH12K_DEVICE_FAMILY_MAX; device_family_id++) {
		driver = ath12k_ahb_family_drivers[device_family_id];
		if (driver) {
			of_id = of_match_device(driver->id_table, &pdev->dev);
			if (of_id) {
				/* Found the driver */
				return device_family_id;
			}
		}
	}

	return ATH12K_DEVICE_FAMILY_MAX;
}

static int ath12k_ahb_probe(struct platform_device *pdev)
{
	enum ath12k_device_family device_id;
	struct ath12k_ahb *ab_ahb;
	struct ath12k_base *ab;
	u32 addr;
	int ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Failed to set 32-bit coherent dma\n");
		return ret;
	}

	ab = ath12k_core_alloc(&pdev->dev, sizeof(struct ath12k_ahb),
			       ATH12K_BUS_AHB);
	if (!ab)
		return -ENOMEM;

	ab_ahb = ath12k_ab_to_ahb(ab);
	ab_ahb->ab = ab;
	spin_lock_init(&ab_ahb->window_lock);
	ab->pdev = pdev;
	platform_set_drvdata(pdev, ab);

	device_id = ath12k_ahb_get_device_family(pdev);
	if (device_id >= ATH12K_DEVICE_FAMILY_MAX) {
		ath12k_err(ab, "failed to get device family: %d\n", device_id);
		ret = -EINVAL;
		goto err_core_free;
	}

	ath12k_dbg(ab, ATH12K_DBG_AHB, "AHB device family id: %d\n", device_id);

	ab_ahb->device_family_ops = &ath12k_ahb_family_drivers[device_id]->ops;

	/* Call device specific probe. This is the callback that can
	 * be used to override any ops in future
	 * probe is validated for NULL during registration.
	 */
	ret = ab_ahb->device_family_ops->probe(pdev);
	if (ret) {
		ath12k_err(ab, "failed to probe device: %d\n", ret);
		goto err_core_free;
	}

	/* Set fixed_mem_region to true for platforms that support fixed memory
	 * reservation from DT. If memory is reserved from DT for FW, ath12k driver
	 * need not to allocate memory.
	 *
	 * WCN7750's memory-region points at an IOVA reservation, not a firmware
	 * carveout, so it must not take the IPQ fixed-memory path.
	 */
	if (!ab->hybrid_bus &&
	    !of_property_read_u32(ab->dev->of_node, "memory-region", &addr))
		set_bit(ATH12K_FLAG_FIXED_MEM_REGION, &ab->dev_flags);

	if (ab->hybrid_bus)
		ath12k_ahb_wcn_parse_host_ddr_ranges(ab);

	ret = ath12k_core_pre_init(ab);
	if (ret)
		goto err_core_free;

	ret = ath12k_ahb_resource_init(ab);
	if (ret)
		goto err_core_free;

	ret = ath12k_hal_srng_init(ab);
	if (ret)
		goto err_resource_deinit;

	ret = ath12k_ce_alloc_pipes(ab);
	if (ret) {
		ath12k_err(ab, "failed to allocate ce pipes: %d\n", ret);
		goto err_hal_srng_deinit;
	}

	ath12k_ahb_init_qmi_ce_config(ab);

	if (ab->hybrid_bus) {
		/* WCN7750: MSI + WPSS remoteproc, no UserPD */
		ret = ath12k_ahb_wcn_get_vregs(ab);
		if (ret)
			goto err_ce_free;

		ret = ath12k_ahb_wcn_pdc_init(ab);
		if (ret)
			goto err_ce_free;

		ret = ath12k_ahb_wcn_setup_msi(ab);
		if (ret)
			goto err_ce_free;

		ret = ath12k_ahb_wcn_get_ep_power_save_state(ab);
		if (ret)
			goto err_ce_free;

		ret = ath12k_ahb_wcn_get_rproc(ab);
		if (ret)
			goto err_ce_free;

		/* before the first rproc_boot() */
		ath12k_ahb_wcn_add_subdev(ab);

		ret = ath12k_pci_config_irq(ab);
		if (ret) {
			ath12k_err(ab, "failed to configure MSI irq: %d\n", ret);
			goto err_wcn_rproc_put;
		}
	} else {
		ret = ath12k_ahb_configure_rproc(ab);
		if (ret)
			goto err_ce_free;

		ret = ath12k_ahb_config_rproc_irq(ab);
		if (ret)
			goto err_rproc_deconfigure;

		ret = ath12k_ahb_config_irq(ab);
		if (ret) {
			ath12k_err(ab, "failed to configure irq: %d\n", ret);
			goto err_rproc_deconfigure;
		}
	}

	/* Invoke arch_init here so that arch-specific init operations
	 * can utilize already initialized ab fields, such as HAL SRNGs.
	 */
	ret = ab_ahb->device_family_ops->arch_init(ab);
	if (ret) {
		ath12k_err(ab, "AHB arch_init failed %d\n", ret);
		goto err_irq_free;
	}

	ret = ath12k_core_init(ab);
	if (ret) {
		ath12k_err(ab, "failed to init core: %d\n", ret);
		goto err_deinit_arch;
	}

	return 0;

err_deinit_arch:
	ab_ahb->device_family_ops->arch_deinit(ab);

err_irq_free:
	if (ab->hybrid_bus) {
		ath12k_pci_free_irq(ab);
		goto err_wcn_rproc_put;
	}

err_rproc_deconfigure:
	mutex_lock(&ath12k_rproc_info_lock);
	ath12k_ahb_deconfigure_rproc(ab);
	mutex_unlock(&ath12k_rproc_info_lock);
	goto err_ce_free;

err_wcn_rproc_put:
	ath12k_ahb_wcn_remove_subdev(ab);
	rproc_put(ab_ahb->wpss_rproc);

err_ce_free:
	ath12k_ce_free_pipes(ab);

err_hal_srng_deinit:
	ath12k_hal_srng_deinit(ab);

err_resource_deinit:
	ath12k_ahb_resource_deinit(ab);

err_core_free:
	ath12k_core_free(ab);
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static void ath12k_ahb_remove_prepare(struct ath12k_base *ab)
{
	unsigned long left;

	if (test_bit(ATH12K_FLAG_RECOVERY, &ab->dev_flags)) {
		left = wait_for_completion_timeout(&ab->driver_recovery,
						   ATH12K_AHB_RECOVERY_TIMEOUT);
		if (!left)
			ath12k_warn(ab, "failed to receive recovery response completion\n");
	}

	set_bit(ATH12K_FLAG_UNREGISTERING, &ab->dev_flags);
	cancel_work_sync(&ab->restart_work);
	cancel_work_sync(&ab->qmi.event_work);
	ath12k_core_stop_cal_restart(ab);
}

static void ath12k_ahb_free_resources(struct ath12k_base *ab)
{
	struct platform_device *pdev = ab->pdev;
	struct ath12k_ahb *ab_ahb = ath12k_ab_to_ahb(ab);

	ath12k_hal_srng_deinit(ab);
	ath12k_ce_free_pipes(ab);
	ath12k_ahb_resource_deinit(ab);
	if (ab->hybrid_bus) {
		ath12k_pci_free_irq(ab);
		ath12k_ahb_wcn_remove_subdev(ab);
		ath12k_ahb_wcn_unmap_msi_addr(ab);
		rproc_put(ab_ahb->wpss_rproc);
	} else {
		mutex_lock(&ath12k_rproc_info_lock);
		ath12k_ahb_deconfigure_rproc(ab);
		mutex_unlock(&ath12k_rproc_info_lock);
	}
	ab_ahb->device_family_ops->arch_deinit(ab);
	ath12k_core_free(ab);
	platform_set_drvdata(pdev, NULL);
}

static void ath12k_ahb_remove(struct platform_device *pdev)
{
	struct ath12k_base *ab = platform_get_drvdata(pdev);

	if (test_bit(ATH12K_FLAG_QMI_FAIL, &ab->dev_flags)) {
		ath12k_core_stop_cal_restart(ab);

		/* ath12k_core_deinit() powers down; a second rproc_shutdown() would strand
		 * the WPSS count.
		 */
		if (!ab->hybrid_bus)
			ath12k_ahb_power_down(ab, false);
		goto qmi_fail;
	}

	ath12k_ahb_remove_prepare(ab);
	ath12k_core_hw_group_cleanup(ab->ag);
qmi_fail:
	ath12k_core_deinit(ab);
	ath12k_ahb_free_resources(ab);
}

int ath12k_ahb_register_driver(const enum ath12k_device_family device_id,
			       struct ath12k_ahb_driver *driver)
{
	struct platform_driver *ahb_driver;

	if (device_id >= ATH12K_DEVICE_FAMILY_MAX)
		return -EINVAL;

	if (!driver || !driver->ops.probe ||
	    !driver->ops.arch_init || !driver->ops.arch_deinit)
		return -EINVAL;

	if (ath12k_ahb_family_drivers[device_id]) {
		pr_err("Driver already registered for id %d\n", device_id);
		return -EALREADY;
	}

	ath12k_ahb_family_drivers[device_id] = driver;

	ahb_driver = &ath12k_ahb_family_drivers[device_id]->driver;
	ahb_driver->driver.name = driver->name;
	ahb_driver->driver.of_match_table = driver->id_table;
	ahb_driver->probe  = ath12k_ahb_probe;
	ahb_driver->remove = ath12k_ahb_remove;

	return platform_driver_register(ahb_driver);
}
EXPORT_SYMBOL(ath12k_ahb_register_driver);

void ath12k_ahb_unregister_driver(const enum ath12k_device_family device_id)
{
	struct platform_driver *ahb_driver;

	if (device_id >= ATH12K_DEVICE_FAMILY_MAX)
		return;

	if (!ath12k_ahb_family_drivers[device_id])
		return;

	ahb_driver = &ath12k_ahb_family_drivers[device_id]->driver;
	platform_driver_unregister(ahb_driver);
	ath12k_ahb_family_drivers[device_id] = NULL;
}
EXPORT_SYMBOL(ath12k_ahb_unregister_driver);
