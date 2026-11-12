// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 * Copyright (c) 2020 Krzysztof Kozlowski <krzk@kernel.org>
 * Author: Sylwester Nawrocki <s.nawrocki@samsung.com>
 * Author: Krzysztof Kozlowski <krzk@kernel.org>
 *
 * Samsung Exynos SoC Adaptive Supply Voltage support
 */

#include <linux/array_size.h>
#include <linux/cpu.h>
#include <linux/device.h>
#include <linux/energy_model.h>
#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-chipid.h>
#include <linux/topology.h>

#include "exynos-asv.h"
#include "exynos5422-asv.h"
#include "exynos7870-asv.h"

#define MHZ 1000000U

/**
 * exynos_asv_update_dev_opps() - Apply an ASV table to the OPPs of a device
 * @asv: ASV data
 * @subsys: subsystem whose table to apply
 * @dev: device owning the OPP table
 *
 * Adjust the voltage of each OPP of @dev that has a level in the table of
 * @subsys. OPPs without a level are left alone.
 */
void exynos_asv_update_dev_opps(struct exynos_asv *asv,
				const struct exynos_asv_subsys *subsys,
				struct device *dev)
{
	struct dev_pm_opp *opp;
	unsigned int opp_freq;
	int i;

	for (i = 0; i < subsys->table.num_rows; i++) {
		struct dev_pm_opp_supply supply;
		unsigned int new_volt, volt, max_volt;
		unsigned long freq;
		int ret;

		opp_freq = exynos_asv_opp_get_frequency(subsys, i);
		freq = (unsigned long)opp_freq * asv->freq_unit;

		opp = dev_pm_opp_find_freq_exact(dev, freq, true);
		if (IS_ERR(opp)) {
			dev_info(asv->dev, "%s opp%d, freq: %u missing\n",
				 dev_name(dev), i, opp_freq);

			continue;
		}

		volt = dev_pm_opp_get_voltage(opp);
		new_volt = asv->opp_get_voltage(subsys, i, volt);
		ret = dev_pm_opp_get_supplies(opp, &supply);
		dev_pm_opp_put(opp);

		if (new_volt == volt)
			continue;

		/*
		 * Keep the OPP's maximum when lowering it: devices on a shared
		 * supply need overlapping ranges to agree on a voltage.
		 */
		max_volt = new_volt;
		if (!ret)
			max_volt = max(new_volt, (unsigned int)supply.u_volt_max);

		ret = dev_pm_opp_adjust_voltage(dev, freq,
						new_volt, new_volt, max_volt);
		if (ret < 0)
			dev_err(asv->dev,
				"Failed to adjust OPP %lu Hz/%u uV for %s\n",
				freq, new_volt, dev_name(dev));
		else
			dev_dbg(asv->dev,
				"Adjusted OPP %lu Hz/%u -> %u uV, %s\n",
				freq, volt, new_volt, dev_name(dev));
	}
}

static int exynos_asv_update_cpu_opps(struct exynos_asv *asv,
				      struct device *cpu)
{
	struct exynos_asv_subsys *subsys = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(asv->subsys); i++) {
		const char *compat = asv->subsys[i].cpu_dt_compat;

		if (compat ? of_device_is_compatible(cpu->of_node, compat) :
			     topology_cluster_id(cpu->id) == asv->subsys[i].id) {
			subsys = &asv->subsys[i];
			break;
		}
	}
	if (!subsys)
		return -EINVAL;

	exynos_asv_update_dev_opps(asv, subsys, cpu);

	return 0;
}

static int exynos_asv_update_opps(struct exynos_asv *asv)
{
	struct opp_table *last_opp_table = NULL;
	struct device *cpu;
	int ret, cpuid;

	for_each_possible_cpu(cpuid) {
		struct opp_table *opp_table;

		cpu = get_cpu_device(cpuid);
		if (!cpu)
			continue;

		opp_table = dev_pm_opp_get_opp_table(cpu);
		if (IS_ERR(opp_table))
			continue;

		if (!last_opp_table || opp_table != last_opp_table) {
			last_opp_table = opp_table;

			ret = exynos_asv_update_cpu_opps(asv, cpu);
			if (!ret) {
				/*
				 * Update EM power values since OPP
				 * voltage values may have changed.
				 */
				em_dev_update_chip_binning(cpu);
			} else {
				dev_err(asv->dev, "Couldn't udate OPPs for cpu%d\n",
					cpuid);
			}
		}

		dev_pm_opp_put_opp_table(opp_table);
	}

	return	0;
}

int exynos_asv_init(struct device *dev, struct regmap *regmap)
{
	int (*probe_func)(struct exynos_asv *asv);
	struct exynos_asv *asv;
	struct device *cpu_dev;
	u32 product_id = 0;
	int ret, i;

	asv = devm_kzalloc_obj(dev, *asv);
	if (!asv)
		return -ENOMEM;

	asv->chipid_regmap = regmap;
	asv->dev = dev;
	asv->freq_unit = MHZ;
	ret = regmap_read(asv->chipid_regmap, EXYNOS_CHIPID_REG_PRO_ID,
			  &product_id);
	if (ret < 0) {
		dev_err(dev, "Cannot read revision from ChipID: %d\n", ret);
		return -ENODEV;
	}

	switch (product_id & EXYNOS_MASK) {
	case 0xE5422000:
		probe_func = exynos5422_asv_init;
		break;
	case 0xE7870000:
		probe_func = exynos7870_asv_init;
		break;
	default:
		dev_dbg(dev, "No ASV support for this SoC\n");
		devm_kfree(dev, asv);
		return 0;
	}

	cpu_dev = get_cpu_device(0);
	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret < 0)
		return -EPROBE_DEFER;

	ret = of_property_read_u32(dev->of_node, "samsung,asv-bin",
				   &asv->of_bin);
	if (ret < 0)
		asv->of_bin = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(asv->subsys); i++)
		asv->subsys[i].asv = asv;

	ret = probe_func(asv);
	if (ret < 0)
		return ret;

	return exynos_asv_update_opps(asv);
}
