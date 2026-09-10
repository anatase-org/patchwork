/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __IRIS_PLATFORM_ELIZA_H__
#define __IRIS_PLATFORM_ELIZA_H__

static const char * const eliza_clk_reset_table[] = { "bus", "core", "vcodec0_core" };

static const char * const eliza_opp_pd_table[] = { "mx", "cx" };

static struct platform_inst_caps platform_inst_cap_eliza = {
	.min_frame_width = 96,
	.max_frame_width = 4096,
	.min_frame_height = 96,
	.max_frame_height = 4096,
	.max_mbpf = (4096 * 2176) / 256,
	.mb_cycles_vpp = 200,
	.mb_cycles_fw = 489583,
	.mb_cycles_fw_vpp = 66234,
	.max_frame_rate = MAXIMUM_FPS,
	.max_operating_rate = MAXIMUM_FPS,
};

#endif
