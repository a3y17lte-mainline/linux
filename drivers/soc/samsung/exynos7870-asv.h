/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung Exynos7870 SoC Adaptive Supply Voltage support
 */

#ifndef __LINUX_SOC_EXYNOS7870_ASV_H
#define __LINUX_SOC_EXYNOS7870_ASV_H

#include <linux/errno.h>

struct exynos_asv;

#ifdef CONFIG_ARM64
int exynos7870_asv_init(struct exynos_asv *asv);
#else
static inline int exynos7870_asv_init(struct exynos_asv *asv)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* __LINUX_SOC_EXYNOS7870_ASV_H */
