// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * Author: Tomasz Figa <t.figa@samsung.com>
 *
 * Clock driver for Exynos clock output
 */

#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#define DRV_NAME			"exynos-clkout"

#define EXYNOS_CLKOUT_NR_CLKS		2
#define EXYNOS_CLKOUT_PARENTS		32

#define EXYNOS_PMU_DEBUG_REG		0xa00
#define EXYNOS_CLKOUT_DISABLE_SHIFT	0
#define EXYNOS_CLKOUT_MUX_SHIFT		8
#define EXYNOS4_CLKOUT_MUX_MASK		0xf
#define EXYNOS5_CLKOUT_MUX_MASK		0x1f

/*
 * Exynos7870: the PMU forwards the oscillator clock to the NFC controller
 * while the controller asserts its clock request line. REQ_POLA selects an
 * active-high request, which the vendor kernel sets for its NFC chip.
 */
#define EXYNOS7870_PMU_NFC_CLK_CTRL	0x600c
#define EXYNOS7870_NFC_CLK_REQ_POLA	4

struct exynos_clkout {
	struct clk_gate gate;
	struct clk_mux mux;
	struct clk_gate nfc_gate;
	spinlock_t slock;
	void __iomem *reg;
	struct device_node *np;
	const struct exynos_clkout_variant *variant;
	u32 pmu_debug_save;
	u32 nfc_clk_save;
	struct clk_hw_onecell_data data;
};

/**
 * struct exynos_clkout_variant - PMU clock outputs of an SoC
 * @mux_mask: CLKOUT mux mask, 0 if the CLKOUT clock is not provided
 * @nfc_clk_reg: offset of the NFC clock control register, 0 if none
 */
struct exynos_clkout_variant {
	u32 mux_mask;
	u32 nfc_clk_reg;
};

static const struct exynos_clkout_variant exynos_clkout_exynos4 = {
	.mux_mask	= EXYNOS4_CLKOUT_MUX_MASK,
};

static const struct exynos_clkout_variant exynos_clkout_exynos5 = {
	.mux_mask	= EXYNOS5_CLKOUT_MUX_MASK,
};

static const struct exynos_clkout_variant exynos_clkout_exynos7870 = {
	.nfc_clk_reg	= EXYNOS7870_PMU_NFC_CLK_CTRL,
};

static const struct of_device_id exynos_clkout_ids[] = {
	{
		.compatible = "samsung,exynos3250-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4210-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4212-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4412-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos5250-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5410-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5420-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5433-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos7870-pmu",
		.data = &exynos_clkout_exynos7870,
	}, { }
};

/*
 * Device will be instantiated as child of PMU device without its own
 * device node.  Therefore match compatibles against parent.
 */
static int exynos_clkout_match_parent_dev(struct device *dev,
					  const struct exynos_clkout_variant **variant)
{
	const struct of_device_id *match;

	if (!dev->parent) {
		dev_err(dev, "not instantiated from MFD\n");
		return -EINVAL;
	}

	/*
	 * 'exynos_clkout_ids' arrays is not the ids array matched by
	 * the dev->parent driver, so of_device_get_match_data() or
	 * device_get_match_data() cannot be used here.
	 */
	match = of_match_device(exynos_clkout_ids, dev->parent);
	if (!match) {
		dev_err(dev, "cannot match parent device\n");
		return -EINVAL;
	}
	*variant = match->data;

	return 0;
}

static struct clk_hw *exynos_clkout_register_clkout(struct exynos_clkout *clkout,
						    struct clk **parents)
{
	const char *parent_names[EXYNOS_CLKOUT_PARENTS];
	int parent_count = 0, i;

	for (i = 0; i < EXYNOS_CLKOUT_PARENTS; ++i) {
		char name[] = "clkoutXX";

		snprintf(name, sizeof(name), "clkout%d", i);
		parents[i] = of_clk_get_by_name(clkout->np, name);
		if (IS_ERR(parents[i])) {
			parent_names[i] = "none";
			continue;
		}

		parent_names[i] = __clk_get_name(parents[i]);
		parent_count = i + 1;
	}

	if (!parent_count)
		return ERR_PTR(-EINVAL);

	clkout->gate.reg = clkout->reg + EXYNOS_PMU_DEBUG_REG;
	clkout->gate.bit_idx = EXYNOS_CLKOUT_DISABLE_SHIFT;
	clkout->gate.flags = CLK_GATE_SET_TO_DISABLE;
	clkout->gate.lock = &clkout->slock;

	clkout->mux.reg = clkout->reg + EXYNOS_PMU_DEBUG_REG;
	clkout->mux.mask = clkout->variant->mux_mask;
	clkout->mux.shift = EXYNOS_CLKOUT_MUX_SHIFT;
	clkout->mux.lock = &clkout->slock;

	return clk_hw_register_composite(NULL, "clkout",
				parent_names, parent_count, &clkout->mux.hw,
				&clk_mux_ops, NULL, NULL, &clkout->gate.hw,
				&clk_gate_ops, CLK_SET_RATE_PARENT
				| CLK_SET_RATE_NO_REPARENT);
}

static struct clk_hw *exynos_clkout_register_nfc(struct exynos_clkout *clkout,
						 struct clk **parent)
{
	struct clk_init_data init = { };
	const char *parent_name;
	int ret;

	*parent = of_clk_get_by_name(clkout->np, "oscclk");
	if (IS_ERR(*parent))
		return ERR_CAST(*parent);

	parent_name = __clk_get_name(*parent);
	init.name = "nfc_clk";
	init.ops = &clk_gate_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clkout->nfc_gate.reg = clkout->reg + clkout->variant->nfc_clk_reg;
	clkout->nfc_gate.bit_idx = EXYNOS7870_NFC_CLK_REQ_POLA;
	clkout->nfc_gate.lock = &clkout->slock;
	clkout->nfc_gate.hw.init = &init;

	ret = clk_hw_register(NULL, &clkout->nfc_gate.hw);

	return ret ? ERR_PTR(ret) : &clkout->nfc_gate.hw;
}

static void exynos_clkout_unregister(struct exynos_clkout *clkout)
{
	int i;

	for (i = 0; i < EXYNOS_CLKOUT_NR_CLKS; i++)
		if (!IS_ERR(clkout->data.hws[i]))
			clk_hw_unregister(clkout->data.hws[i]);
}

static int exynos_clkout_probe(struct platform_device *pdev)
{
	struct clk *parents[EXYNOS_CLKOUT_PARENTS + 1];
	struct exynos_clkout *clkout;
	int ret, i;

	clkout = devm_kzalloc_flex(&pdev->dev, *clkout, data.hws,
				   EXYNOS_CLKOUT_NR_CLKS);
	if (!clkout)
		return -ENOMEM;

	ret = exynos_clkout_match_parent_dev(&pdev->dev, &clkout->variant);
	if (ret)
		return ret;

	clkout->np = pdev->dev.of_node;
	if (!clkout->np) {
		/*
		 * pdev->dev.parent was checked by exynos_clkout_match_parent_dev()
		 * so it is not NULL.
		 */
		clkout->np = pdev->dev.parent->of_node;
	}

	platform_set_drvdata(pdev, clkout);

	spin_lock_init(&clkout->slock);

	for (i = 0; i < ARRAY_SIZE(parents); i++)
		parents[i] = ERR_PTR(-ENOENT);

	clkout->reg = of_iomap(clkout->np, 0);
	if (!clkout->reg)
		return -ENODEV;

	clkout->data.num = clkout->variant->nfc_clk_reg ? 2 : 1;
	clkout->data.hws[0] = ERR_PTR(-ENOENT);
	clkout->data.hws[1] = ERR_PTR(-ENOENT);

	if (clkout->variant->mux_mask) {
		clkout->data.hws[0] = exynos_clkout_register_clkout(clkout, parents);
		if (IS_ERR(clkout->data.hws[0])) {
			ret = PTR_ERR(clkout->data.hws[0]);
			goto err_unreg;
		}
	}

	if (clkout->variant->nfc_clk_reg) {
		clkout->data.hws[1] =
			exynos_clkout_register_nfc(clkout, &parents[EXYNOS_CLKOUT_PARENTS]);
		if (IS_ERR(clkout->data.hws[1])) {
			ret = PTR_ERR(clkout->data.hws[1]);
			goto err_unreg;
		}
	}

	ret = of_clk_add_hw_provider(clkout->np, of_clk_hw_onecell_get, &clkout->data);
	if (ret)
		goto err_unreg;

	return 0;

err_unreg:
	exynos_clkout_unregister(clkout);
	iounmap(clkout->reg);
	for (i = 0; i < ARRAY_SIZE(parents); ++i)
		if (!IS_ERR(parents[i]))
			clk_put(parents[i]);

	dev_err(&pdev->dev, "failed to register clkout clock\n");

	return ret;
}

static void exynos_clkout_remove(struct platform_device *pdev)
{
	struct exynos_clkout *clkout = platform_get_drvdata(pdev);

	of_clk_del_provider(clkout->np);
	exynos_clkout_unregister(clkout);
	iounmap(clkout->reg);
}

static int exynos_clkout_suspend(struct device *dev)
{
	struct exynos_clkout *clkout = dev_get_drvdata(dev);

	if (clkout->variant->mux_mask)
		clkout->pmu_debug_save = readl(clkout->reg + EXYNOS_PMU_DEBUG_REG);
	if (clkout->variant->nfc_clk_reg)
		clkout->nfc_clk_save = readl(clkout->reg + clkout->variant->nfc_clk_reg);

	return 0;
}

static int exynos_clkout_resume(struct device *dev)
{
	struct exynos_clkout *clkout = dev_get_drvdata(dev);

	if (clkout->variant->mux_mask)
		writel(clkout->pmu_debug_save, clkout->reg + EXYNOS_PMU_DEBUG_REG);
	if (clkout->variant->nfc_clk_reg)
		writel(clkout->nfc_clk_save, clkout->reg + clkout->variant->nfc_clk_reg);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(exynos_clkout_pm_ops, exynos_clkout_suspend,
			 exynos_clkout_resume);

static struct platform_driver exynos_clkout_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = pm_sleep_ptr(&exynos_clkout_pm_ops),
	},
	.probe = exynos_clkout_probe,
	.remove = exynos_clkout_remove,
};
module_platform_driver(exynos_clkout_driver);

MODULE_AUTHOR("Krzysztof Kozlowski <krzk@kernel.org>");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("Samsung Exynos clock output driver");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL");
