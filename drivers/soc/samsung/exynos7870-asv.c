// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos7870 SoC Adaptive Supply Voltage support
 *
 * The CPU and GPU voltage of each OPP depends on the ASV group the chip was
 * binned into and on the version of the voltage tables it was fused for.
 * Both are kept in fuses that only the secure world can read. The tables
 * below come from the ECT (Exynos Characteristic Table) the bootloader of
 * the vendor firmware holds in RAM; the CPU ones are the same for both
 * clusters.
 *
 * The GPU's OPP table only exists once its driver has probed, so the GPU
 * voltages are adjusted when the GPU device gets bound.
 */

#include <linux/arm-smccc.h>
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/units.h>

#include "exynos-asv.h"
#include "exynos7870-asv.h"

/* Vendor EL3 monitor call reading a secure-only register */
#define EXYNOS7870_SMC_CMD_REG		((unsigned long)(u32)-101)
#define EXYNOS7870_SMC_REG_SFR_READ	(0x3UL << 30)

#define EXYNOS7870_ASV_INFO_BASE	0x100d9000
#define EXYNOS7870_ASV_INFO_CPU		0	/* CPUCL0 [15:0], CPUCL1 [31:16] */
#define EXYNOS7870_ASV_INFO_G3D		1	/* [15:0] */
#define EXYNOS7870_ASV_INFO_VERSION	3

/* Fields of each 16-bit domain entry */
#define EXYNOS7870_ASV_GROUP		GENMASK(3, 0)
#define EXYNOS7870_ASV_MOD_GROUP	GENMASK(7, 4)	/* signed */
#define EXYNOS7870_ASV_SSA10		GENMASK(9, 8)
#define EXYNOS7870_ASV_SSA11		GENMASK(11, 10)
#define EXYNOS7870_ASV_SSA0		GENMASK(15, 12)
#define EXYNOS7870_ASV_TABLE_VER	GENMASK(6, 0)
#define EXYNOS7870_ASV_FUSED		BIT(7)

#define E7870_ASV_NUM_TABLES		7
#define E7870_ASV_CPU_NUM_LEVELS	13
#define E7870_ASV_GPU_NUM_LEVELS	7
#define E7870_ASV_NUM_GROUPS		16

/* Table entries are 6.25 mV steps above 600 mV */
#define E7870_ASV_VOLT_BASE		600000
#define E7870_ASV_VOLT_STEP		6250

/*
 * SSA fuses: the faster levels (CPU: above 902.2 MHz, GPU: 676 MHz and
 * above) add the SSA10 offset, the others the SSA11 offset, and SSA0 raises
 * the voltage floor in 25 mV steps.
 */
#define E7870_ASV_CPU_SSA1_SPLIT	6
#define E7870_ASV_GPU_SSA1_SPLIT	4
#define E7870_ASV_SSA0_BASE		575000
#define E7870_ASV_SSA0_STEP		25000

static const unsigned int exynos7870_asv_ssa1_offset[] = {
	0, 12500, 25000, 50000
};

/* CPU levels, in kHz */
static const unsigned int exynos7870_asv_cpu_freq[E7870_ASV_CPU_NUM_LEVELS] = {
	1586000, 1482000, 1352000, 1248000, 1144000, 1014000, 902200,
	838500, 756600, 676000, 546000, 448500, 342875,
};

static const u8 exynos7870_asv_cpu_volt[E7870_ASV_NUM_TABLES][E7870_ASV_CPU_NUM_LEVELS]
				      [E7870_ASV_NUM_GROUPS] = {
	[0] = {
		{ 92, 90, 88, 86, 84, 82, 80, 78, 76, 74, 72, 70, 68, 66, 64, 62 },
		{ 82, 80, 78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 56, 54, 52 },
		{ 72, 70, 68, 66, 64, 62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42 },
		{ 64, 62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34 },
		{ 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26 },
		{ 48, 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18 },
		{ 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12 },
		{ 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6 },
		{ 31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11,  9,  7,  5,  3,  1 },
		{ 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6,  4,  2,  0,  0,  0 },
		{ 21, 19, 17, 15, 13, 11,  9,  7,  5,  3,  1,  0,  0,  0,  0,  0 },
		{ 16, 14, 12, 10,  8,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0 },
		{ 12, 10,  8,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[1] = {
		{ 80, 78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 56, 54, 52, 50 },
		{ 70, 68, 66, 64, 62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40 },
		{ 63, 61, 59, 57, 55, 53, 51, 49, 47, 45, 43, 41, 39, 37, 35, 33 },
		{ 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32, 30, 28 },
		{ 50, 48, 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20 },
		{ 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16 },
		{ 44, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14 },
		{ 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8 },
		{ 33, 31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11,  9,  7,  5,  3 },
		{ 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6,  4,  2,  0,  0 },
		{ 23, 21, 19, 17, 15, 13, 11,  9,  7,  5,  3,  1,  0,  0,  0,  0 },
		{ 18, 16, 14, 12, 10,  8,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0 },
		{ 14, 12, 10,  8,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[2] = {
		{ 80, 78, 76, 74, 70, 68, 66, 64, 62, 60, 56, 54, 50, 48, 46, 46 },
		{ 68, 68, 66, 62, 60, 60, 58, 56, 54, 52, 48, 48, 44, 42, 40, 40 },
		{ 60, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 34 },
		{ 54, 52, 50, 48, 46, 46, 44, 42, 40, 38, 36, 34, 32, 30, 28, 28 },
		{ 46, 44, 42, 40, 40, 38, 36, 34, 32, 32, 30, 28, 26, 24, 22, 22 },
		{ 40, 38, 36, 34, 32, 32, 30, 28, 28, 26, 24, 22, 20, 18, 16, 16 },
		{ 34, 34, 32, 30, 28, 28, 26, 24, 24, 22, 20, 18, 16, 14, 12, 12 },
		{ 26, 26, 24, 22, 20, 20, 18, 16, 16, 14, 12, 10,  8,  6,  6,  4 },
		{ 22, 20, 20, 18, 16, 14, 14, 12, 10, 10,  8,  6,  4,  4,  2,  2 },
		{ 18, 16, 16, 14, 12, 10, 10,  8,  6,  6,  4,  2,  0,  0,  0,  0 },
		{ 12, 10, 10,  8,  6,  4,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0 },
		{ 12, 10,  8,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[3] = {
		{ 86, 84, 83, 77, 75, 71, 68, 65, 62, 59, 56, 53, 49, 46, 43, 41 },
		{ 77, 75, 74, 69, 66, 63, 60, 57, 54, 51, 48, 45, 42, 39, 36, 35 },
		{ 69, 68, 66, 62, 59, 56, 53, 50, 47, 45, 42, 39, 36, 33, 30, 29 },
		{ 61, 60, 59, 54, 52, 49, 47, 44, 41, 38, 36, 33, 30, 27, 25, 23 },
		{ 53, 52, 50, 46, 44, 41, 39, 36, 34, 31, 29, 26, 23, 21, 18, 17 },
		{ 47, 45, 44, 40, 38, 35, 33, 31, 28, 26, 23, 21, 18, 16, 13, 12 },
		{ 42, 41, 39, 35, 33, 31, 28, 26, 24, 21, 19, 17, 14, 12,  9,  8 },
		{ 32, 31, 30, 26, 24, 22, 20, 18, 16, 13, 11,  9,  7,  5,  3,  1 },
		{ 28, 27, 26, 22, 21, 19, 17, 15, 13, 11,  9,  7,  5,  3,  1,  0 },
		{ 23, 22, 21, 18, 16, 14, 12, 11,  9,  7,  5,  3,  1,  0,  0,  0 },
		{ 15, 14, 13, 10,  9,  7,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0 },
		{ 10,  9,  9,  6,  5,  3,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[4] = {
		{ 86, 84, 83, 77, 75, 71, 68, 65, 62, 59, 56, 53, 49, 46, 43, 41 },
		{ 77, 75, 74, 69, 66, 63, 60, 57, 54, 51, 48, 45, 42, 39, 36, 35 },
		{ 67, 66, 64, 60, 57, 55, 52, 49, 46, 43, 40, 37, 35, 32, 29, 27 },
		{ 60, 59, 58, 53, 51, 48, 45, 43, 40, 37, 34, 32, 29, 26, 24, 22 },
		{ 53, 52, 50, 46, 44, 41, 39, 36, 34, 31, 29, 26, 23, 21, 18, 17 },
		{ 44, 43, 42, 38, 36, 33, 31, 29, 26, 24, 22, 19, 17, 14, 12, 11 },
		{ 37, 36, 34, 31, 29, 26, 24, 22, 20, 17, 15, 13, 11,  8,  6,  5 },
		{ 32, 31, 30, 26, 24, 22, 20, 18, 16, 13, 11,  9,  7,  5,  3,  1 },
		{ 28, 27, 26, 22, 21, 19, 17, 15, 13, 11,  9,  7,  5,  3,  1,  0 },
		{ 23, 22, 21, 18, 16, 14, 12, 11,  9,  7,  5,  3,  1,  0,  0,  0 },
		{ 15, 14, 13, 10,  9,  7,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0 },
		{ 10,  9,  9,  6,  5,  3,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[5] = {
		{ 84, 83, 81, 76, 74, 71, 68, 65, 62, 60, 57, 54, 51, 48, 45, 44 },
		{ 75, 74, 72, 68, 65, 63, 60, 57, 55, 52, 49, 47, 44, 41, 38, 37 },
		{ 67, 66, 64, 60, 57, 55, 52, 49, 46, 43, 40, 37, 35, 32, 29, 27 },
		{ 60, 59, 58, 53, 51, 48, 45, 43, 40, 37, 34, 32, 29, 26, 24, 22 },
		{ 52, 51, 49, 45, 43, 41, 38, 36, 34, 31, 29, 26, 24, 21, 19, 18 },
		{ 44, 43, 42, 38, 36, 33, 31, 29, 26, 24, 22, 19, 17, 14, 12, 11 },
		{ 37, 36, 34, 31, 29, 26, 24, 22, 20, 17, 15, 13, 11,  8,  6,  5 },
		{ 32, 31, 30, 26, 24, 22, 20, 18, 16, 13, 11,  9,  7,  5,  3,  1 },
		{ 28, 27, 26, 23, 21, 19, 17, 15, 13, 11,  9,  7,  5,  2,  0,  0 },
		{ 24, 23, 22, 18, 17, 15, 13, 11,  9,  7,  5,  3,  1,  0,  0,  0 },
		{ 16, 16, 15, 11, 10,  8,  6,  4,  3,  1,  0,  0,  0,  0,  0,  0 },
		{ 12, 11, 10,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  8,  7,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[6] = {
		{ 84, 83, 81, 76, 74, 71, 68, 65, 62, 60, 57, 54, 51, 48, 45, 44 },
		{ 75, 74, 72, 68, 65, 63, 60, 57, 55, 52, 49, 47, 44, 41, 38, 37 },
		{ 67, 66, 64, 60, 57, 55, 52, 49, 46, 43, 40, 37, 35, 32, 29, 27 },
		{ 60, 59, 58, 53, 51, 48, 45, 43, 40, 37, 34, 32, 29, 26, 24, 22 },
		{ 52, 51, 49, 45, 43, 41, 38, 36, 34, 31, 29, 26, 24, 21, 19, 18 },
		{ 44, 43, 42, 38, 36, 33, 31, 29, 26, 24, 22, 19, 17, 14, 12, 11 },
		{ 37, 36, 34, 31, 29, 26, 24, 22, 20, 17, 15, 13, 11,  8,  6,  5 },
		{ 32, 31, 30, 26, 24, 22, 20, 18, 16, 13, 11,  9,  7,  5,  3,  1 },
		{ 28, 27, 26, 23, 21, 19, 17, 15, 13, 11,  9,  7,  5,  2,  0,  0 },
		{ 24, 23, 22, 18, 17, 15, 13, 11,  9,  7,  5,  3,  1,  0,  0,  0 },
		{ 16, 16, 15, 11, 10,  8,  6,  4,  3,  1,  0,  0,  0,  0,  0,  0 },
		{ 12, 11, 10,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  8,  7,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
};

/* GPU levels, in kHz (the vendor's 1146 MHz and above are disabled) */
static const unsigned int exynos7870_asv_gpu_freq[E7870_ASV_GPU_NUM_LEVELS] = {
	1001000, 845000, 728000, 676000, 545000, 450000, 343000,
};

static const u8 exynos7870_asv_gpu_volt[E7870_ASV_NUM_TABLES][E7870_ASV_GPU_NUM_LEVELS]
				      [E7870_ASV_NUM_GROUPS] = {
	[0] = {
		{ 60, 58, 57, 56, 54, 52, 50, 48, 47, 45, 43, 41, 39, 38, 36, 34 },
		{ 52, 50, 49, 48, 46, 44, 42, 40, 39, 37, 35, 33, 31, 30, 28, 26 },
		{ 44, 42, 40, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16 },
		{ 32, 30, 29, 28, 26, 24, 23, 21, 20, 18, 17, 15, 14, 12, 11,  9 },
		{ 24, 22, 21, 20, 18, 17, 16, 15, 13, 12, 11, 10,  9,  7,  6,  5 },
		{ 24, 22, 21, 20, 18, 17, 16, 15, 13, 12, 11, 10,  9,  7,  6,  5 },
		{ 24, 22, 21, 20, 18, 17, 16, 15, 13, 12, 11, 10,  9,  7,  6,  5 },
	},
	[1] = {
		{ 56, 54, 53, 52, 50, 48, 46, 44, 43, 41, 39, 37, 35, 34, 32, 30 },
		{ 48, 46, 45, 44, 42, 40, 38, 36, 35, 33, 31, 29, 27, 26, 24, 22 },
		{ 40, 38, 36, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12 },
		{ 28, 26, 25, 24, 22, 20, 19, 17, 16, 14, 13, 11, 10,  8,  7,  5 },
		{ 20, 18, 17, 16, 14, 13, 12, 11,  9,  8,  7,  6,  5,  3,  2,  1 },
		{ 20, 18, 17, 16, 14, 13, 12, 11,  9,  8,  7,  6,  5,  3,  2,  1 },
		{ 20, 18, 17, 16, 14, 13, 12, 11,  9,  8,  7,  6,  5,  3,  2,  1 },
	},
	[2] = {
		{ 64, 62, 60, 58, 54, 52, 50, 48, 46, 44, 42, 38, 36, 34, 32, 30 },
		{ 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 30, 28, 26, 24, 22, 20 },
		{ 40, 40, 38, 34, 32, 32, 30, 28, 26, 24, 22, 20, 16, 14, 14, 12 },
		{ 34, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 10,  8,  8,  6 },
		{ 24, 22, 20, 18, 16, 16, 14, 12, 10,  8,  6,  4,  2,  2,  0,  0 },
		{ 14, 12, 10,  8,  8,  6,  4,  2,  2,  0,  0,  0,  0,  0,  0,  0 },
		{ 12, 10,  8,  6,  4,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[3] = {
		{ 64, 63, 60, 56, 54, 52, 49, 47, 45, 42, 40, 38, 36, 33, 31, 30 },
		{ 48, 46, 44, 41, 38, 36, 34, 32, 30, 28, 26, 24, 22, 19, 17, 16 },
		{ 36, 35, 33, 29, 27, 25, 23, 21, 19, 18, 16, 14, 12, 10,  8,  7 },
		{ 32, 31, 29, 25, 23, 21, 20, 18, 16, 14, 12, 10,  8,  6,  4,  3 },
		{ 22, 21, 19, 16, 14, 12, 11,  9,  7,  5,  3,  2,  0,  0,  0,  0 },
		{ 15, 14, 12,  9,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0 },
		{  7,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[4] = {
		{ 64, 63, 60, 56, 54, 52, 49, 47, 45, 42, 40, 38, 36, 33, 31, 30 },
		{ 48, 46, 44, 41, 38, 36, 34, 32, 30, 28, 26, 24, 22, 19, 17, 16 },
		{ 36, 35, 33, 29, 27, 25, 23, 21, 19, 18, 16, 14, 12, 10,  8,  7 },
		{ 32, 31, 29, 25, 23, 21, 20, 18, 16, 14, 12, 10,  8,  6,  4,  3 },
		{ 22, 21, 19, 16, 14, 12, 11,  9,  7,  5,  3,  2,  0,  0,  0,  0 },
		{ 15, 14, 12,  9,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0 },
		{  7,  6,  4,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[5] = {
		{ 65, 64, 61, 57, 55, 53, 50, 48, 46, 44, 41, 39, 37, 34, 32, 31 },
		{ 49, 48, 46, 42, 40, 38, 35, 33, 31, 29, 27, 24, 22, 20, 18, 17 },
		{ 38, 36, 34, 31, 28, 26, 24, 22, 20, 18, 16, 14, 11,  9,  7,  6 },
		{ 33, 32, 30, 27, 25, 23, 21, 19, 17, 15, 13, 11,  8,  6,  4,  3 },
		{ 23, 22, 20, 17, 15, 13, 12, 10,  8,  6,  4,  2,  0,  0,  0,  0 },
		{ 15, 14, 12,  9,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  7,  5,  3,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
	[6] = {
		{ 65, 64, 61, 57, 55, 53, 50, 48, 46, 44, 41, 39, 37, 34, 32, 31 },
		{ 49, 48, 46, 42, 40, 38, 35, 33, 31, 29, 27, 24, 22, 20, 18, 17 },
		{ 38, 36, 34, 31, 28, 26, 24, 22, 20, 18, 16, 14, 11,  9,  7,  6 },
		{ 33, 32, 30, 27, 25, 23, 21, 19, 17, 15, 13, 11,  8,  6,  4,  3 },
		{ 23, 22, 20, 17, 15, 13, 12, 10,  8,  6,  4,  2,  0,  0,  0,  0 },
		{ 15, 14, 12,  9,  7,  6,  4,  2,  1,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  7,  5,  3,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
	},
};

struct exynos7870_asv_domain {
	const unsigned int *freq;
	const u8 *volt;		/* [table][level][group] */
	unsigned int num_levels;
	unsigned int ssa1_split;
};

static const struct exynos7870_asv_domain exynos7870_asv_cpu = {
	.freq = exynos7870_asv_cpu_freq,
	.volt = &exynos7870_asv_cpu_volt[0][0][0],
	.num_levels = E7870_ASV_CPU_NUM_LEVELS,
	.ssa1_split = E7870_ASV_CPU_SSA1_SPLIT,
};

static const struct exynos7870_asv_domain exynos7870_asv_gpu = {
	.freq = exynos7870_asv_gpu_freq,
	.volt = &exynos7870_asv_gpu_volt[0][0][0],
	.num_levels = E7870_ASV_GPU_NUM_LEVELS,
	.ssa1_split = E7870_ASV_GPU_SSA1_SPLIT,
};

struct exynos7870_asv_gpu {
	struct exynos_asv *asv;
	struct exynos_asv_subsys subsys;
	struct notifier_block nb;
};

static int exynos7870_asv_read_info(unsigned int word, u32 *val)
{
	unsigned long addr = EXYNOS7870_ASV_INFO_BASE + word * 4;
	struct arm_smccc_res res;

	arm_smccc_smc(EXYNOS7870_SMC_CMD_REG,
		      EXYNOS7870_SMC_REG_SFR_READ | (addr >> 2),
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EIO;

	*val = res.a2;
	return 0;
}

static int exynos7870_asv_opp_get_voltage(const struct exynos_asv_subsys *subsys,
					  int level, unsigned int volt)
{
	return exynos_asv_opp_get_voltage(subsys, level, 0);
}

static int exynos7870_asv_init_subsys(struct exynos_asv *asv,
				      struct exynos_asv_subsys *subsys,
				      const struct exynos7870_asv_domain *dom,
				      const char *name, u32 info)
{
	unsigned int ssa10 = FIELD_GET(EXYNOS7870_ASV_SSA10, info);
	unsigned int ssa11 = FIELD_GET(EXYNOS7870_ASV_SSA11, info);
	unsigned int ssa0 = FIELD_GET(EXYNOS7870_ASV_SSA0, info);
	unsigned int floor = E7870_ASV_SSA0_BASE + ssa0 * E7870_ASV_SSA0_STEP;
	const u8 *volt_table;
	int group;
	u32 *buf;
	int lv;

	group = FIELD_GET(EXYNOS7870_ASV_GROUP, info) +
		sign_extend32(FIELD_GET(EXYNOS7870_ASV_MOD_GROUP, info), 3);
	if (group < 0 || group >= E7870_ASV_NUM_GROUPS) {
		dev_warn(asv->dev, "%s: invalid ASV group %d\n", name, group);
		return 0;
	}

	buf = devm_kzalloc_objs(asv->dev, *buf, dom->num_levels * 2);
	if (!buf)
		return -ENOMEM;

	volt_table = dom->volt +
		     asv->table * dom->num_levels * E7870_ASV_NUM_GROUPS;

	for (lv = 0; lv < dom->num_levels; lv++) {
		unsigned int volt;

		volt = E7870_ASV_VOLT_BASE + E7870_ASV_VOLT_STEP *
		       volt_table[lv * E7870_ASV_NUM_GROUPS + group];
		if (lv < dom->ssa1_split)
			volt += exynos7870_asv_ssa1_offset[ssa10];
		else
			volt += exynos7870_asv_ssa1_offset[ssa11];

		buf[lv * 2] = dom->freq[lv];
		buf[lv * 2 + 1] = max(volt, floor);
	}

	subsys->table.num_rows = dom->num_levels;
	subsys->table.num_cols = 2;
	subsys->table.buf = buf;

	dev_info(asv->dev, "%s: ASV group %d, table version %u\n",
		 name, group, asv->table);
	return 0;
}

static bool exynos7870_asv_is_gpu(struct device *dev)
{
	return of_device_is_compatible(dev->of_node, "samsung,exynos7870-mali");
}

static void exynos7870_asv_update_gpu(struct exynos7870_asv_gpu *gpu,
				      struct device *dev)
{
	/* Only with the GPU driver's OPP table in place */
	if (dev_pm_opp_get_opp_count(dev) <= 0)
		return;

	exynos_asv_update_dev_opps(gpu->asv, &gpu->subsys, dev);
}

static int exynos7870_asv_gpu_notify(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct exynos7870_asv_gpu *gpu = container_of(nb, typeof(*gpu), nb);
	struct device *dev = data;

	if (action == BUS_NOTIFY_BOUND_DRIVER && exynos7870_asv_is_gpu(dev))
		exynos7870_asv_update_gpu(gpu, dev);

	return NOTIFY_DONE;
}

static int exynos7870_asv_update_bound_gpu(struct device *dev, void *data)
{
	if (exynos7870_asv_is_gpu(dev) && device_is_bound(dev))
		exynos7870_asv_update_gpu(data, dev);

	return 0;
}

static void exynos7870_asv_gpu_unregister(void *data)
{
	struct exynos7870_asv_gpu *gpu = data;

	bus_unregister_notifier(&platform_bus_type, &gpu->nb);
}

static int exynos7870_asv_init_gpu(struct exynos_asv *asv, u32 info)
{
	struct exynos7870_asv_gpu *gpu;
	int ret;

	gpu = devm_kzalloc_obj(asv->dev, *gpu);
	if (!gpu)
		return -ENOMEM;

	gpu->asv = asv;
	gpu->subsys.asv = asv;
	ret = exynos7870_asv_init_subsys(asv, &gpu->subsys, &exynos7870_asv_gpu,
					 "GPU", info);
	if (ret || !gpu->subsys.table.buf)
		return ret;

	gpu->nb.notifier_call = exynos7870_asv_gpu_notify;
	ret = bus_register_notifier(&platform_bus_type, &gpu->nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(asv->dev, exynos7870_asv_gpu_unregister,
				       gpu);
	if (ret)
		return ret;

	/* The GPU may have been bound before the notifier was registered */
	return bus_for_each_dev(&platform_bus_type, NULL, gpu,
				exynos7870_asv_update_bound_gpu);
}

int exynos7870_asv_init(struct exynos_asv *asv)
{
	u32 cpu_info, gpu_info, ver_info;
	char name[16];
	int ret, i;

	/* Both clusters are Cortex-A53, so match them by cluster id */
	for (i = 0; i < ARRAY_SIZE(asv->subsys); i++)
		asv->subsys[i].id = i;

	asv->freq_unit = HZ_PER_KHZ;
	asv->opp_get_voltage = exynos7870_asv_opp_get_voltage;

	ret = exynos7870_asv_read_info(EXYNOS7870_ASV_INFO_CPU, &cpu_info);
	if (!ret)
		ret = exynos7870_asv_read_info(EXYNOS7870_ASV_INFO_G3D,
					       &gpu_info);
	if (!ret)
		ret = exynos7870_asv_read_info(EXYNOS7870_ASV_INFO_VERSION,
					       &ver_info);
	if (ret) {
		dev_warn(asv->dev, "Cannot read ASV fuses, keeping DT voltages\n");
		return 0;
	}

	asv->table = FIELD_GET(EXYNOS7870_ASV_TABLE_VER, ver_info);
	if (!(ver_info & EXYNOS7870_ASV_FUSED) ||
	    asv->table >= E7870_ASV_NUM_TABLES) {
		dev_info(asv->dev, "No ASV data (%#x), keeping DT voltages\n",
			 ver_info);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(asv->subsys); i++) {
		snprintf(name, sizeof(name), "CPU cluster %d", i);
		ret = exynos7870_asv_init_subsys(asv, &asv->subsys[i],
						 &exynos7870_asv_cpu, name,
						 cpu_info >> (16 * i));
		if (ret)
			return ret;
	}

	return exynos7870_asv_init_gpu(asv, gpu_info & 0xffff);
}
