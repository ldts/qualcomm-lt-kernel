// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,wcss-qcs404.h>

#include "clk-regmap.h"
#include "clk-branch.h"
#include "common.h"
#include "reset.h"

/* Q6SSTOP clocks. These clocks are voted
 * by remoteproc client when loaded from
 * user space, system hang is seen when CCF turns
 * off unused clocks. As a temp solution use
 * CLK_IGNORE_UNUSED flags which prevent these
 * clocks from being gated during bootup.
 */
static struct clk_branch lcc_ahbfabric_cbc_clk = {
	.halt_reg = 0x1b004,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x1b004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lcc_ahbfabric_cbc_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct clk_branch lcc_q6ss_ahbs_cbc_clk = {
	.halt_reg = 0x22000,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x22000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lcc_q6ss_ahbs_cbc_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct clk_branch lcc_q6ss_tcm_slave_cbc_clk = {
	.halt_reg = 0x1c000,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x1c000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lcc_q6ss_tcm_slave_cbc_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct clk_branch lcc_q6ss_ahbm_cbc_clk = {
	.halt_reg = 0x22004,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x22004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lcc_q6ss_ahbm_cbc_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct clk_branch lcc_q6ss_axim_cbc_clk = {
	.halt_reg = 0x1c004,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x1c004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lcc_q6ss_axim_cbc_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct clk_branch lcc_q6ss_bcr_sleep_clk = {
	.halt_reg = 0x6004,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x6004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "lcc_q6ss_bcr_sleep_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

/* TCSR clock */
static struct clk_branch wcss_lcc_csr_cbcr_clk = {
	.halt_reg = 0x8008,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x8008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "wcss_lcc_csr_cbcr_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

/* Q6SSTOP_QDSP6SS clock */
static struct clk_branch q6ss_xo_clk = {
	.halt_reg = 0x38,
	/* CLK_OFF would not toggle until WCSS is out of reset */
	.halt_check = BRANCH_HALT_SKIP,
	.clkr = {
		.enable_reg = 0x38,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "q6ss_xo_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct clk_branch q6ss_slp_clk = {
	.halt_reg = 0x3c,
	/* CLK_OFF would not toggle until WCSS is out of reset */
	.halt_check = BRANCH_HALT_SKIP,
	.clkr = {
		.enable_reg = 0x3c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "q6ss_slp_clk",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct clk_branch q6sstop_q6ss_gfmux_clk_src = {
	.halt_reg = 0x20,
	.halt_check = BRANCH_VOTED,
	.clkr = {
		.enable_reg = 0x20,
		.enable_mask = BIT(1) | BIT(3) | BIT(8),
		.hw.init = &(struct clk_init_data){
			.name = "q6sstop_q6ss_gfmux_clk_src",
			.ops = &clk_branch2_ops,
			.flags = CLK_IGNORE_UNUSED,
		},
	},
};

static struct regmap_config wcss_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.fast_io	= true,
};

static struct clk_regmap *wcss_q6sstop_qcs404_clocks[] = {
	[WCSS_AHBFABRIC_CBCR_CLK] = &lcc_ahbfabric_cbc_clk.clkr,
	[WCSS_AHBS_CBCR_CLK] = &lcc_q6ss_ahbs_cbc_clk.clkr,
	[WCSS_TCM_CBCR_CLK] = &lcc_q6ss_tcm_slave_cbc_clk.clkr,
	[WCSS_AHBM_CBCR_CLK] = &lcc_q6ss_ahbm_cbc_clk.clkr,
	[WCSS_AXIM_CBCR_CLK] = &lcc_q6ss_axim_cbc_clk.clkr,
	[WCSS_BCR_CBCR_CLK] = &lcc_q6ss_bcr_sleep_clk.clkr,
};

static const struct qcom_reset_map qdsp6ss_qcs404_resets[] = {
	[Q6SSTOP_QDSP6SS_RESET] = {0x14, 0},
	[Q6SSTOP_QDSP6SS_CORE_RESET] = {0x14, 1},
	[Q6SSTOP_QDSP6SS_BUS_RESET] = {0x14, 2},
	[Q6SSTOP_CORE_ARCH_RESET] = {0x14, 12},
};

static const struct qcom_reset_map q6sstop_qcs404_resets[] = {
	[Q6SSTOP_BCR_RESET] = {0x6000},
};

static const struct qcom_cc_desc wcss_q6sstop_qcs404_desc = {
	.config = &wcss_regmap_config,
	.clks = wcss_q6sstop_qcs404_clocks,
	.num_clks = ARRAY_SIZE(wcss_q6sstop_qcs404_clocks),
	.resets = q6sstop_qcs404_resets,
	.num_resets = ARRAY_SIZE(q6sstop_qcs404_resets),
};

static struct clk_regmap *wcnss_tcsr_qcs404_clocks[] = {
	[WCSS_LCC_CBCR_CLK] = &wcss_lcc_csr_cbcr_clk.clkr,
};

static const struct qcom_cc_desc wcnss_tcsr_qcs404_desc = {
	.config = &wcss_regmap_config,
	.clks = wcnss_tcsr_qcs404_clocks,
	.num_clks = ARRAY_SIZE(wcnss_tcsr_qcs404_clocks),
};

static struct clk_regmap *wcnss_qdsp6ss_qcs404_clocks[] = {
	[WCSS_QDSP6SS_XO_CBCR_CLK] = &q6ss_xo_clk.clkr,
	[WCSS_QDSP6SS_SLEEP_CBCR_CLK] = &q6ss_slp_clk.clkr,
	[WCSS_QDSP6SS_GFMMUX_CLK] = &q6sstop_q6ss_gfmux_clk_src.clkr,
};

static const struct qcom_cc_desc wcnss_qdsp6ss_qcs404_desc = {
	.config = &wcss_regmap_config,
	.clks = wcnss_qdsp6ss_qcs404_clocks,
	.num_clks = ARRAY_SIZE(wcnss_qdsp6ss_qcs404_clocks),
	.resets = qdsp6ss_qcs404_resets,
	.num_resets = ARRAY_SIZE(qdsp6ss_qcs404_resets),
};

static const struct of_device_id wcss_cc_qcs404_match_table[] = {
	{ .compatible = "qcom,qcs404-wcsscc" },
	{ }
};
MODULE_DEVICE_TABLE(of, wcss_cc_qcs404_match_table);

static int wcss_cc_qcs404_probe(struct platform_device *pdev)
{
	const struct qcom_cc_desc *desc;
	int ret;

	wcss_regmap_config.name = "wcss_q6sstop";
	desc = &wcss_q6sstop_qcs404_desc;

	ret = qcom_cc_probe_by_index(pdev, 0, desc);
	if (ret)
		return ret;

	wcss_regmap_config.name = "wcnss_tcsr";
	desc = &wcnss_tcsr_qcs404_desc;

	ret = qcom_cc_probe_by_index(pdev, 1, desc);
	if (ret)
		return ret;

	wcss_regmap_config.name = "wcss_qdsp6ss";
	desc = &wcnss_qdsp6ss_qcs404_desc;

	return qcom_cc_probe_by_index(pdev, 2, desc);
}

static struct platform_driver wcss_cc_qcs404_driver = {
	.probe		= wcss_cc_qcs404_probe,
	.driver		= {
		.name	= "qcs404-wcsscc",
		.of_match_table = wcss_cc_qcs404_match_table,
	},
};

static int __init wcss_cc_qcs404_init(void)
{
	return platform_driver_register(&wcss_cc_qcs404_driver);
}
subsys_initcall(wcss_cc_qcs404_init);

static void __exit wcss_cc_qcs404_exit(void)
{
	platform_driver_unregister(&wcss_cc_qcs404_driver);
}
module_exit(wcss_cc_qcs404_exit);

MODULE_DESCRIPTION("QTI WCSS_CC QCS404 Driver");
MODULE_LICENSE("GPL v2");
