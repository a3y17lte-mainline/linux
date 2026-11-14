// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos7870 PMU: system sleep (suspend-to-RAM)
 *
 * The firmware (PSCI 0.1, no SYSTEM_SUSPEND) powers the system down on a
 * CPU_SUSPEND request at affinity level 3 from the last online CPU. It only
 * saves its own state and executes WFI: the PMU central sequencer then takes
 * the SoC down according to the per-block SYS_PWR registers, which the
 * kernel has to program, together with the wakeup masks and the sequencer
 * options. Wakeup is a warm reset through the boot ROM and the firmware back
 * to the PSCI resume address.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/printk.h>
#include <linux/psci.h>
#include <linux/suspend.h>
#include <linux/soc/samsung/exynos-pmu.h>

#include <asm/cputype.h>

#include "exynos-pmu.h"

#define EXYNOS7870_UP_SCHEDULER			0x0120
#define EXYNOS7870_CENTRAL_SEQ_CONFIGURATION	0x0200
#define EXYNOS7870_CENTRAL_SEQ_OPTION		0x0208
#define EXYNOS7870_CENTRAL_SEQ_OPTION1		0x020c
#define EXYNOS7870_CENTRAL_SEQ_MIF_CONFIGURATION	0x0240
#define EXYNOS7870_CENTRAL_SEQ_MIF_OPTION	0x0248
#define EXYNOS7870_RESET_LPI_TIMEOUT		0x0420
#define EXYNOS7870_WAKEUP_STAT			0x0600
#define EXYNOS7870_WAKEUP_STAT2			0x0604
#define EXYNOS7870_WAKEUP_STAT3			0x0608
#define EXYNOS7870_EINT_WAKEUP_MASK		0x060c
#define EXYNOS7870_WAKEUP_MASK			0x0610
#define EXYNOS7870_WAKEUP_MASK2			0x0614
#define EXYNOS7870_WAKEUP_MASK3			0x0618
#define EXYNOS7870_WAKEUP_MASK_MIF		0x0628
#define EXYNOS7870_CPU_OPTION(n)		(0x2008 + (n) * 0x80)
#define EXYNOS7870_CPU_CONFIGURATION(cl, n)	(0x2000 + (((cl) << 2) | (n)) * 0x80)
#define EXYNOS7870_CPU_LOCAL_PWR_CFG		GENMASK(3, 0)
#define EXYNOS7870_CPU_DURATION0(n)		(0x2010 + (n) * 0x80)
#define EXYNOS7870_NONCPU_OPTION(cl)		(0x2408 + (cl) * 0x20)
#define EXYNOS7870_NONCPU_DURATION0(cl)		(0x2410 + (cl) * 0x20)
#define EXYNOS7870_L2_OPTION(cl)		(0x2608 + (cl) * 0x20)
#define EXYNOS7870_G3D_OPTION			0x4008
#define EXYNOS7870_G3D_DURATION0		0x4010
#define EXYNOS7870_DISPAUD_OPTION		0x4028
#define EXYNOS7870_ISP_OPTION			0x4048
#define EXYNOS7870_MFCMSCL_OPTION		0x4068
#define EXYNOS7870_MEMORY_G3D_OPTION		0x4a08
#define EXYNOS7870_TOP_PWR_OPTION		0x2c48
#define EXYNOS7870_TOP_BUS_MIF_OPTION		0x2c88
#define EXYNOS7870_TOP_PWR_MIF_OPTION		0x2cc8
#define EXYNOS7870_XXTI_DURATION3		0x343c
#define EXYNOS7870_TCXO_DURATION3		0x347c
#define EXYNOS7870_EXT_REGULATOR_DURATION3	0x361c
#define EXYNOS7870_EXT_REGULATOR_MIF_DURATION3	0x363c

/* CENTRAL_SEQ(_MIF)_CONFIGURATION: the sequencer runs while this bit is 0 */
#define EXYNOS7870_CENTRAL_SEQ_PWR_CFG		BIT(16)
/* Set by the vendor kernel in WAKEUP_MASK(_MIF) for the system sleep only */
#define EXYNOS7870_WAKEUP_MASK_SLEEP		BIT(30)

/* Power state options and counters */
#define EXYNOS7870_OPT_SC_COUNTER		BIT(0)
#define EXYNOS7870_OPT_SC_FEEDBACK		BIT(1)
#define EXYNOS7870_OPT_MEMPWRDOWN_FEEDBACK	BIT(3)
#define EXYNOS7870_OPT_USE_RETENTION		BIT(4)
#define EXYNOS7870_OPT_STANDBYWFI		BIT(16)
#define EXYNOS7870_OPT_STANDBYWFIL2		BIT(16)
#define EXYNOS7870_OPT_AUTOMATIC_L2FLUSHREQ	BIT(17)
#define EXYNOS7870_OPT_STANDBYWFE		BIT(24)
#define EXYNOS7870_OPT_SMPEN			BIT(28)
#define EXYNOS7870_DURATION0_SC			GENMASK(11, 4)

/* CPU_SUSPEND power_state for the whole system: affinity level 3, power down */
#define EXYNOS7870_PSCI_SYSTEM_SLEEP		0x03010000

/* Vendor wakeup masks for the system sleep: EINT, PMIC and CP sources stay on */
#define EXYNOS7870_SLEEP_WAKEUP_MASK		0x50006e1e
#define EXYNOS7870_SLEEP_WAKEUP_MASK2		0xffff00ff
#define EXYNOS7870_SLEEP_WAKEUP_MASK3		0xffff00ff

#define SLP(_offset, _val)	{ .offset = (_offset), .val = { [SYS_SLEEP] = (_val) } }

/* Per-block power configuration during system sleep (SYS_PWR_REG registers) */
static const struct exynos_pmu_conf exynos7870_pmu_config[] = {
	SLP(0x1000, 0x8),	/* CPUCL0_CPU0 */
	SLP(0x1004, 0x0),	/* DIS_IRQ_CPUCL0_CPU0_LOCAL */
	SLP(0x1008, 0x0),	/* DIS_IRQ_CPUCL0_CPU0_CENTRAL */
	SLP(0x100c, 0x0),	/* DIS_IRQ_CPUCL0_CPU0_CPUSEQUENCER */
	SLP(0x1010, 0x8),	/* CPUCL0_CPU1 */
	SLP(0x1014, 0x0),	/* DIS_IRQ_CPUCL0_CPU1_LOCAL */
	SLP(0x1018, 0x0),	/* DIS_IRQ_CPUCL0_CPU1_CENTRAL */
	SLP(0x101c, 0x0),	/* DIS_IRQ_CPUCL0_CPU1_CPUSEQUENCER */
	SLP(0x1020, 0x8),	/* CPUCL0_CPU2 */
	SLP(0x1024, 0x0),	/* DIS_IRQ_CPUCL0_CPU2_LOCAL */
	SLP(0x1028, 0x0),	/* DIS_IRQ_CPUCL0_CPU2_CENTRAL */
	SLP(0x102c, 0x0),	/* DIS_IRQ_CPUCL0_CPU2_CPUSEQUENCER */
	SLP(0x1030, 0x8),	/* CPUCL0_CPU3 */
	SLP(0x1034, 0x0),	/* DIS_IRQ_CPUCL0_CPU3_LOCAL */
	SLP(0x1038, 0x0),	/* DIS_IRQ_CPUCL0_CPU3_CENTRAL */
	SLP(0x103c, 0x0),	/* DIS_IRQ_CPUCL0_CPU3_CPUSEQUENCER */
	SLP(0x1040, 0x8),	/* CPUCL1_CPU0 */
	SLP(0x1044, 0x0),	/* DIS_IRQ_CPUCL1_CPU0_LOCAL */
	SLP(0x1048, 0x0),	/* DIS_IRQ_CPUCL1_CPU0_CENTRAL */
	SLP(0x104c, 0x0),	/* DIS_IRQ_CPUCL1_CPU0_CPUSEQUENCER */
	SLP(0x1050, 0x8),	/* CPUCL1_CPU1 */
	SLP(0x1054, 0x0),	/* DIS_IRQ_CPUCL1_CPU1_LOCAL */
	SLP(0x1058, 0x0),	/* DIS_IRQ_CPUCL1_CPU1_CENTRAL */
	SLP(0x105c, 0x0),	/* DIS_IRQ_CPUCL1_CPU1_CPUSEQUENCER */
	SLP(0x1060, 0x8),	/* CPUCL1_CPU2 */
	SLP(0x1064, 0x0),	/* DIS_IRQ_CPUCL1_CPU2_LOCAL */
	SLP(0x1068, 0x0),	/* DIS_IRQ_CPUCL1_CPU2_CENTRAL */
	SLP(0x106c, 0x0),	/* DIS_IRQ_CPUCL1_CPU2_CPUSEQUENCER */
	SLP(0x1070, 0x8),	/* CPUCL1_CPU3 */
	SLP(0x1074, 0x0),	/* DIS_IRQ_CPUCL1_CPU3_LOCAL */
	SLP(0x1078, 0x0),	/* DIS_IRQ_CPUCL1_CPU3_CENTRAL */
	SLP(0x107c, 0x0),	/* DIS_IRQ_CPUCL1_CPU3_CPUSEQUENCER */
	SLP(0x1080, 0x8),	/* CPUCL0_NONCPU */
	SLP(0x1084, 0x8),	/* CPUCL1_NONCPU */
	SLP(0x10c0, 0x7),	/* CPUCL0_L2 */
	SLP(0x10c4, 0x7),	/* CPUCL1_L2 */
	SLP(0x1100, 0x0),	/* CLKSTOP_CMU_TOP */
	SLP(0x1104, 0x0),	/* CLKRUN_CMU_TOP */
	SLP(0x1108, 0x3),	/* RETENTION_CMU_TOP */
	SLP(0x110c, 0x0),	/* RESET_CMU_TOP */
	SLP(0x111c, 0x0),	/* RESET_CPUCLKSTOP */
	SLP(0x1120, 0x0),	/* CLKSTOP_CMU_MIF */
	SLP(0x1124, 0x0),	/* CLKRUN_CMU_MIF */
	SLP(0x1128, 0x3),	/* RETENTION_CMU_MIF */
	SLP(0x112c, 0x0),	/* RESET_CMU_MIF */
	SLP(0x1134, 0x1),	/* DDRPHY_ISO */
	SLP(0x1138, 0x0),	/* DDRPHY_DLL_CLK */
	SLP(0x1140, 0x0),	/* DISABLE_PLL_CMU_TOP */
	SLP(0x1160, 0x0),	/* DISABLE_PLL_CMU_MIF */
	SLP(0x1170, 0x3),	/* RESET_AHEAD_CP */
	SLP(0x1174, 0x3),	/* RESET_AHEAD_GNSS */
	SLP(0x1180, 0x0),	/* TOP_BUS */
	SLP(0x1184, 0x3),	/* TOP_RETENTION */
	SLP(0x1188, 0x3),	/* TOP_PWR */
	SLP(0x1190, 0x0),	/* TOP_BUS_MIF */
	SLP(0x1194, 0x3),	/* TOP_RETENTION_MIF */
	SLP(0x1198, 0x3),	/* TOP_PWR_MIF */
	SLP(0x11a0, 0x0),	/* LOGIC_RESET */
	SLP(0x11a4, 0x1),	/* OSCCLK_GATE */
	SLP(0x11a8, 0x0),	/* SLEEP_RESET */
	SLP(0x11b0, 0x0),	/* LOGIC_RESET_MIF */
	SLP(0x11b4, 0x1),	/* OSCCLK_GATE_MIF */
	SLP(0x11b8, 0x0),	/* SLEEP_RESET_MIF */
	SLP(0x11bc, 0x3),	/* RESET_ASB_MIF_GNSS */
	SLP(0x11c0, 0x3),	/* MEMORY_TOP */
	SLP(0x11c4, 0x1),	/* TCXO_GATE_GNSS */
	SLP(0x11c8, 0x3),	/* RESET_ASB_GNSS */
	SLP(0x11cc, 0x1),	/* CLEANY_BUS_CP */
	SLP(0x11d0, 0x3),	/* LOGIC_RESET_CP */
	SLP(0x11d4, 0x1),	/* TCXO_GATE_CP */
	SLP(0x11d8, 0x3),	/* RESET_ASB_CP */
	SLP(0x11dc, 0x3),	/* RESET_ASB_MIF_CP */
	SLP(0x11e0, 0x0),	/* MEMORY_MIF_ALIVEIRAM */
	SLP(0x11e4, 0x3),	/* MEMORY_MIF_TOP */
	SLP(0x11e8, 0x1),	/* CLEANY_BUS_GNSS */
	SLP(0x11ec, 0x3),	/* LOGIC_RESET_GNSS */
	SLP(0x1200, 0x0),	/* PAD_RETENTION_LPDDR3 */
	SLP(0x1204, 0x0),	/* PAD_RETENTION_AUD */
	SLP(0x1218, 0x0),	/* PAD_RETENTION_MMC2 */
	SLP(0x1220, 0x0),	/* PAD_RETENTION_TOP */
	SLP(0x1224, 0x0),	/* PAD_RETENTION_UART */
	SLP(0x1228, 0x0),	/* PAD_RETENTION_MMC0 */
	SLP(0x122c, 0x0),	/* PAD_RETENTION_MMC1 */
	SLP(0x1230, 0x0),	/* PAD_RETENTION_SPI */
	SLP(0x1234, 0x0),	/* PAD_RETENTION_MIF */
	SLP(0x1240, 0x1),	/* PAD_ISOLATION */
	SLP(0x1248, 0x0),	/* PAD_RETENTION_BOOTLDO */
	SLP(0x1250, 0x1),	/* PAD_ISOLATION_MIF */
	SLP(0x1260, 0x0),	/* PAD_ALV_SEL */
	SLP(0x128c, 0x0),	/* TCXO */
	SLP(0x12c0, 0x0),	/* EXT_REGULATOR */
	SLP(0x12c4, 0x0),	/* EXT_REGULATOR_MIF */
	SLP(0x1300, 0x0),	/* GPIO_MODE */
	SLP(0x1320, 0x0),	/* GPIO_MODE_MIF */
	SLP(0x1340, 0x0),	/* GPIO_MODE_DISPAUD */
	SLP(0x1380, 0x0),	/* CLKSTOP_OPEN_CMU_TOP */
	SLP(0x1384, 0x0),	/* CLKSTOP_OPEN_CMU_MIF */
	SLP(0x1388, 0x0),	/* CLKSTOP_OPEN_CMU_G3D */
	SLP(0x138c, 0x0),	/* CLKSTOP_OPEN_CMU_DISPAUD */
	SLP(0x1390, 0x0),	/* CLKSTOP_OPEN_CMU_ISP */
	SLP(0x1394, 0x0),	/* CLKSTOP_OPEN_CMU_MFCMSCL */
	SLP(0x1400, 0x0),	/* G3D */
	SLP(0x1404, 0x0),	/* DISPAUD */
	SLP(0x1408, 0x0),	/* ISP */
	SLP(0x140c, 0x0),	/* MFCMSCL */
	SLP(0x1440, 0x0),	/* CLKRUN_CMU_G3D */
	SLP(0x1444, 0x0),	/* CLKRUN_CMU_DISPAUD */
	SLP(0x1448, 0x0),	/* CLKRUN_CMU_ISP */
	SLP(0x144c, 0x0),	/* CLKRUN_CMU_MFCMSCL */
	SLP(0x1480, 0x0),	/* CLKSTOP_CMU_G3D */
	SLP(0x1484, 0x0),	/* CLKSTOP_CMU_DISPAUD */
	SLP(0x1488, 0x0),	/* CLKSTOP_CMU_ISP */
	SLP(0x148c, 0x0),	/* CLKSTOP_CMU_MFCMSCL */
	SLP(0x14c0, 0x0),	/* DISABLE_PLL_CMU_G3D */
	SLP(0x14c4, 0x0),	/* DISABLE_PLL_CMU_DISPAUD */
	SLP(0x14c8, 0x0),	/* DISABLE_PLL_CMU_ISP */
	SLP(0x14cc, 0x0),	/* DISABLE_PLL_CMU_MFCMSCL */
	SLP(0x1500, 0x0),	/* RESET_LOGIC_G3D */
	SLP(0x1504, 0x0),	/* RESET_LOGIC_DISPAUD */
	SLP(0x1508, 0x0),	/* RESET_LOGIC_ISP */
	SLP(0x150c, 0x0),	/* RESET_LOGIC_MFCMSCL */
	SLP(0x1540, 0x0),	/* MEMORY_G3D */
	SLP(0x1544, 0x0),	/* MEMORY_DISPAUD */
	SLP(0x1548, 0x0),	/* MEMORY_ISP */
	SLP(0x154c, 0x0),	/* MEMORY_MFCMSCL */
	SLP(0x1580, 0x0),	/* RESET_CMU_G3D */
	SLP(0x1584, 0x0),	/* RESET_CMU_DISPAUD */
	SLP(0x1588, 0x0),	/* RESET_CMU_ISP */
	SLP(0x158c, 0x0),	/* RESET_CMU_MFCMSCL */
	{ PMU_TABLE_END, },
};

/*
 * Options the vendor kernel programs at boot for the system power-down
 * sequences. The bootloader leaves them at their reset values, which are
 * known to work for runtime CPU, cluster and domain power gating, so they
 * are only applied for the duration of a system sleep and restored after.
 */
struct exynos7870_sleep_option {
	u32 offset;
	u32 clear;
	u32 set;
};

#define OPT(_offset, _clear, _set)					\
	{ .offset = (_offset), .clear = (_clear), .set = (_set) }

#define CPU_OPT(n)							\
	OPT(EXYNOS7870_CPU_OPTION(n),					\
	    EXYNOS7870_OPT_STANDBYWFE | EXYNOS7870_OPT_SC_FEEDBACK,	\
	    EXYNOS7870_OPT_SMPEN | EXYNOS7870_OPT_STANDBYWFI |		\
	    EXYNOS7870_OPT_MEMPWRDOWN_FEEDBACK | EXYNOS7870_OPT_SC_COUNTER)

#define CPU_DURATION(n)							\
	OPT(EXYNOS7870_CPU_DURATION0(n), EXYNOS7870_DURATION0_SC, 0x330)

#define FEEDBACK_OPT(reg)						\
	OPT(reg, EXYNOS7870_OPT_SC_COUNTER, EXYNOS7870_OPT_SC_FEEDBACK)

static const struct exynos7870_sleep_option exynos7870_sleep_options[] = {
	CPU_OPT(0), CPU_OPT(1), CPU_OPT(2), CPU_OPT(3),
	CPU_OPT(4), CPU_OPT(5), CPU_OPT(6), CPU_OPT(7),
	CPU_DURATION(0), CPU_DURATION(1), CPU_DURATION(2), CPU_DURATION(3),
	CPU_DURATION(4), CPU_DURATION(5), CPU_DURATION(6), CPU_DURATION(7),
	OPT(EXYNOS7870_NONCPU_OPTION(0), EXYNOS7870_OPT_SC_FEEDBACK,
	    EXYNOS7870_OPT_SC_COUNTER),
	OPT(EXYNOS7870_NONCPU_OPTION(1), EXYNOS7870_OPT_SC_FEEDBACK,
	    EXYNOS7870_OPT_SC_COUNTER),
	OPT(EXYNOS7870_NONCPU_DURATION0(0), EXYNOS7870_DURATION0_SC, 0x330),
	OPT(EXYNOS7870_NONCPU_DURATION0(1), EXYNOS7870_DURATION0_SC, 0x330),
	OPT(EXYNOS7870_L2_OPTION(0), EXYNOS7870_OPT_AUTOMATIC_L2FLUSHREQ |
	    EXYNOS7870_OPT_USE_RETENTION, EXYNOS7870_OPT_STANDBYWFIL2),
	OPT(EXYNOS7870_L2_OPTION(1), EXYNOS7870_OPT_AUTOMATIC_L2FLUSHREQ |
	    EXYNOS7870_OPT_USE_RETENTION, EXYNOS7870_OPT_STANDBYWFIL2),
	FEEDBACK_OPT(EXYNOS7870_TOP_PWR_OPTION),
	FEEDBACK_OPT(EXYNOS7870_TOP_PWR_MIF_OPTION),
	FEEDBACK_OPT(EXYNOS7870_DISPAUD_OPTION),
	FEEDBACK_OPT(EXYNOS7870_MFCMSCL_OPTION),
	FEEDBACK_OPT(EXYNOS7870_ISP_OPTION),
	FEEDBACK_OPT(EXYNOS7870_G3D_OPTION),
	FEEDBACK_OPT(EXYNOS7870_MEMORY_G3D_OPTION),
	OPT(EXYNOS7870_RESET_LPI_TIMEOUT, 0, BIT(0)),
	OPT(EXYNOS7870_UP_SCHEDULER, 0, GENMASK(1, 0)),
	/* Oscillator and external regulator stabilisation: 1, 1, 2.5, 2 ms */
	OPT(EXYNOS7870_XXTI_DURATION3, ~0U, 0x658),
	OPT(EXYNOS7870_TCXO_DURATION3, ~0U, 0x658),
	OPT(EXYNOS7870_EXT_REGULATOR_DURATION3, ~0U, 0xfde),
	OPT(EXYNOS7870_EXT_REGULATOR_MIF_DURATION3, ~0U, 0xcb1),
	OPT(EXYNOS7870_G3D_DURATION0, EXYNOS7870_DURATION0_SC, 0),
};

static u32 exynos7870_sleep_saved[ARRAY_SIZE(exynos7870_sleep_options)];

static void exynos7870_pmu_update(u32 offset, u32 clear, u32 set)
{
	pmu_raw_writel((pmu_raw_readl(offset) & ~clear) | set, offset);
}

static void exynos7870_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int i;

	if (mode != SYS_SLEEP)
		return;

	for (i = 0; i < ARRAY_SIZE(exynos7870_sleep_options); i++) {
		const struct exynos7870_sleep_option *opt =
			&exynos7870_sleep_options[i];

		exynos7870_sleep_saved[i] = pmu_raw_readl(opt->offset);
		exynos7870_pmu_update(opt->offset, opt->clear, opt->set);
	}

	pmu_raw_writel(EXYNOS7870_SLEEP_WAKEUP_MASK | EXYNOS7870_WAKEUP_MASK_SLEEP,
		       EXYNOS7870_WAKEUP_MASK);
	pmu_raw_writel(EXYNOS7870_SLEEP_WAKEUP_MASK2, EXYNOS7870_WAKEUP_MASK2);
	pmu_raw_writel(EXYNOS7870_SLEEP_WAKEUP_MASK3, EXYNOS7870_WAKEUP_MASK3);
	exynos7870_pmu_update(EXYNOS7870_WAKEUP_MASK_MIF, 0,
			      EXYNOS7870_WAKEUP_MASK_SLEEP);
}

static void exynos7870_powerdown_conf_extra(enum sys_powerdown mode)
{
	if (mode != SYS_SLEEP)
		return;

	pmu_raw_writel(0x00ff0000, EXYNOS7870_CENTRAL_SEQ_OPTION);
	pmu_raw_writel(0x10000000, EXYNOS7870_CENTRAL_SEQ_OPTION1);
	pmu_raw_writel(0x00000010, EXYNOS7870_CENTRAL_SEQ_MIF_OPTION);
	pmu_raw_writel(0, EXYNOS7870_TOP_BUS_MIF_OPTION);

	exynos7870_pmu_update(EXYNOS7870_CENTRAL_SEQ_CONFIGURATION,
			      EXYNOS7870_CENTRAL_SEQ_PWR_CFG, 0);
	exynos7870_pmu_update(EXYNOS7870_CENTRAL_SEQ_MIF_CONFIGURATION,
			      EXYNOS7870_CENTRAL_SEQ_PWR_CFG, 0);
}

/* Undo the sleep configuration, after a wakeup or an aborted entry */
static void exynos7870_powerdown_exit(void)
{
	unsigned int i;

	exynos7870_pmu_update(EXYNOS7870_CENTRAL_SEQ_CONFIGURATION, 0,
			      EXYNOS7870_CENTRAL_SEQ_PWR_CFG);
	exynos7870_pmu_update(EXYNOS7870_CENTRAL_SEQ_MIF_CONFIGURATION, 0,
			      EXYNOS7870_CENTRAL_SEQ_PWR_CFG);
	pmu_raw_writel(0, EXYNOS7870_TOP_BUS_MIF_OPTION);
	exynos7870_pmu_update(EXYNOS7870_WAKEUP_MASK,
			      EXYNOS7870_WAKEUP_MASK_SLEEP, 0);

	for (i = 0; i < ARRAY_SIZE(exynos7870_sleep_options); i++)
		pmu_raw_writel(exynos7870_sleep_saved[i],
			       exynos7870_sleep_options[i].offset);
}

#if defined(CONFIG_SUSPEND) && defined(CONFIG_CPU_IDLE) && defined(CONFIG_ARM_PSCI_FW)
static int exynos7870_suspend_enter(suspend_state_t state)
{
	u64 mpidr = read_cpuid_mpidr();
	u32 cpu_cfg = EXYNOS7870_CPU_CONFIGURATION(MPIDR_AFFINITY_LEVEL(mpidr, 1),
						   MPIDR_AFFINITY_LEVEL(mpidr, 0));
	int ret;

	exynos_sys_powerdown_conf(SYS_SLEEP);

	/*
	 * The CPU PM notifier has cleared this CPU's local power configuration
	 * on the way here, as for a core power-down idle state. In system sleep
	 * the central sequencer powers the CPU down according to its SYS_PWR
	 * register, as in the vendor kernel, so keep the local configuration on.
	 * The CPU PM exit notifier sets it again in any case.
	 */
	pmu_raw_writel(pmu_raw_readl(cpu_cfg) | EXYNOS7870_CPU_LOCAL_PWR_CFG,
		       cpu_cfg);

	pm_pr_dbg("Entering system sleep, CPU_CONFIGURATION %08x WAKEUP_MASK %08x EINT %08x\n",
		  pmu_raw_readl(cpu_cfg),
		  pmu_raw_readl(EXYNOS7870_WAKEUP_MASK),
		  pmu_raw_readl(EXYNOS7870_EINT_WAKEUP_MASK));

	pm_set_resume_via_firmware();
	ret = psci_cpu_suspend_enter(EXYNOS7870_PSCI_SYSTEM_SLEEP);

	exynos7870_powerdown_exit();

	pr_info("PM: %s, WAKEUP_STAT %08x %08x %08x\n",
		ret ? "sleep aborted" : "woke up",
		pmu_raw_readl(EXYNOS7870_WAKEUP_STAT),
		pmu_raw_readl(EXYNOS7870_WAKEUP_STAT2),
		pmu_raw_readl(EXYNOS7870_WAKEUP_STAT3));

	return ret;
}

static const struct platform_suspend_ops exynos7870_suspend_ops = {
	.enter	= exynos7870_suspend_enter,
	.valid	= suspend_valid_only_mem,
};

static void exynos7870_pmu_init(void)
{
	suspend_set_ops(&exynos7870_suspend_ops);
}
#else
static void exynos7870_pmu_init(void)
{
}
#endif

const struct exynos_pmu_data exynos7870_pmu_data = {
	.pmu_config		= exynos7870_pmu_config,
	.pmu_cpu_pwr_cfg	= true,
	.pmu_cluster_seq	= true,
	.pmu_init		= exynos7870_pmu_init,
	.powerdown_conf		= exynos7870_powerdown_conf,
	.powerdown_conf_extra	= exynos7870_powerdown_conf_extra,
};
