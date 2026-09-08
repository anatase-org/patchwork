/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#ifndef ATH12K_CORE_WIFI7_H
#define ATH12K_CORE_WIFI7_H

struct ath12k_pci_reg_base;
extern const struct ath12k_pci_reg_base ath12k_wifi7_reg_base;

int ath12k_wifi7_arch_init(struct ath12k_base *ab);
void ath12k_wifi7_arch_deinit(struct ath12k_base *ab);

#endif
