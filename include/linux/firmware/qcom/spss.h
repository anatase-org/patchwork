/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_SPSS_H
#define __QCOM_SPSS_H

#include <linux/rpmsg.h>

int qcom_spss_nvm_link(struct device *consumer);
struct rpmsg_endpoint *qcom_spss_tpm_attach(struct device *consumer,
					 rpmsg_rx_cb_t callback, void *priv);
void qcom_spss_tpm_detach(void *priv);

#endif
