// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos7870 audio mixer (S1402X)
 *
 * The mixer sits between the SoC I2S controller (application processor
 * input, IN1) and the external codec (the AIF4 I2S bus, on which the mixer
 * is always the bus master at 64 fs with 24-bit slots). It also drives the
 * codec master clock on its MCKO pin.
 *
 * Only the application processor path is supported so far. The modem (IN2)
 * and Bluetooth (IN3) inputs are configured with safe defaults but not
 * exposed.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

/* 8-bit registers at a 4-byte stride */
#define AM_SOFT_RSTB		0x000
#define  AM_SOFT_RSTB_DATA	BIT(1)
#define  AM_SOFT_RSTB_SYS	BIT(0)
#define AM_IN1_CTL1		0x004
#define AM_IN1_CTL2		0x008
#define AM_IN2_CTL2		0x014
#define AM_IN3_CTL1		0x01c
#define AM_IN3_CTL2		0x020
#define AM_IN3_CTL3		0x024
#define  AM_INCTL1_MASTER	BIT(4)
#define  AM_INCTL1_SLOT		GENMASK(3, 2)
#define  AM_INCTL2_XFS		GENMASK(6, 5)
#define  AM_INCTL2_LRCK_POL	BIT(4)
#define  AM_INCTL2_DF		GENMASK(3, 2)
#define  AM_INCTL2_DL		GENMASK(1, 0)
#define  AM_INCTL3_DAD		GENMASK(6, 5)
#define  AM_INCTL3_DF		GENMASK(3, 0)
#define AM_HQ_CTL		0x028
#define  AM_HQ_CTL_MCKO_EN	BIT(2)
#define  AM_HQ_CTL_HQ_EN	BIT(0)
#define AM_SLOT_L		0x02c
#define AM_SLOT_R		0x030
#define AM_TSLOT		0x038
#define AM_DIG_EN		0x03c
#define  AM_DIG_EN_MIX_SHIFT	3
#define AM_DMIX1		0x040
#define  AM_DMIX1_EN1_SHIFT	3
#define  AM_DMIX1_LVL1_SHIFT	0
#define AM_DMIX2		0x044
#define AM_DOUTMX1		0x058
#define  AM_DOUTMX1_SEL1_SHIFT	0
#define AM_DOUTMX2		0x05c
#define AM_INAMP_CTL		0x060
#define AM_OUTAP1_CTL		0x064
#define AM_OUTCP1_CTL		0x068
#define AM_ALC_CTL		0x1a0
#define AM_ALC_SGR		0x1c8

#define AM_XFS_32FS		0
#define AM_XFS_48FS		1
#define AM_XFS_64FS		2
#define AM_DL_16BIT		0
#define AM_DL_24BIT		3

/* PMU register holding the mixer power control, active low */
#define EXYNOS7870_PMU_AUD_PATH_CFG	0x0080
#define  AUD_PATH_CFG_MIXER_OFF		BIT(0)

#define AM_SYSCLK_FS		512
#define AM_AUTOSUSPEND_MS	500

enum {
	AM_DAI_AP,
	AM_DAI_CODEC,
};

struct exynos7870_audmixer {
	struct device *dev;
	struct regmap *regmap;
	struct regmap *pmu;
	struct clk *pclk;
	struct clk *sysclk;
	struct reset_control *rst;
	struct clk_hw mclk;
	unsigned int bclk_ratio;
};

static bool am_accessible_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AM_SOFT_RSTB ... AM_DMIX2:
	case AM_DOUTMX1 ... AM_OUTCP1_CTL:
	case AM_ALC_CTL ... AM_ALC_SGR:
		return true;
	default:
		return false;
	}
}

static bool am_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == AM_SOFT_RSTB;
}

static const struct regmap_config am_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = AM_ALC_SGR,
	.readable_reg = am_accessible_reg,
	.writeable_reg = am_accessible_reg,
	.volatile_reg = am_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
};

/*
 * Default configuration, from the vendor driver: all inputs I2S, 16-bit,
 * 32 fs; the Bluetooth input is a consumer with 32-bit slots and uses
 * slots 1 and 2 of 2; the amplifier feedback input is 24-bit at 64 fs.
 * The application processor input goes to the digital mixer at 0 dB and
 * the codec feeds the capture path of the application processor.
 */
static const struct reg_sequence am_init_seq[] = {
	{ AM_IN1_CTL2, FIELD_PREP_CONST(AM_INCTL2_XFS, AM_XFS_32FS) },
	{ AM_IN2_CTL2, FIELD_PREP_CONST(AM_INCTL2_XFS, AM_XFS_32FS) },
	{ AM_IN3_CTL1, FIELD_PREP_CONST(AM_INCTL1_SLOT, 1) },
	{ AM_IN3_CTL2, FIELD_PREP_CONST(AM_INCTL2_XFS, AM_XFS_32FS) },
	{ AM_IN3_CTL3, FIELD_PREP_CONST(AM_INCTL3_DAD, 1) |
		       FIELD_PREP_CONST(AM_INCTL3_DF, 4) },
	{ AM_SLOT_L, 0x1 },
	{ AM_SLOT_R, 0x2 },
	{ AM_TSLOT, 0x1 },
	{ AM_INAMP_CTL, 0x2a },
	{ AM_OUTAP1_CTL, 0x00 },
	{ AM_OUTCP1_CTL, 0x00 },
	{ AM_HQ_CTL, 0x00 },
	{ AM_DMIX1, BIT(AM_DMIX1_EN1_SHIFT) },
	{ AM_DOUTMX1, 1 << AM_DOUTMX1_SEL1_SHIFT },
};

static int am_reset_data(struct exynos7870_audmixer *am)
{
	int ret;

	ret = regmap_write(am->regmap, AM_SOFT_RSTB, AM_SOFT_RSTB_SYS);
	if (ret)
		return ret;

	return regmap_write(am->regmap, AM_SOFT_RSTB,
			    AM_SOFT_RSTB_SYS | AM_SOFT_RSTB_DATA);
}

/* 0, -2.87, -6.02, -9.28, -12.04, -14.54, -18.06, -20.56 dB, inverted */
static const DECLARE_TLV_DB_RANGE(am_mix_tlv,
	0, 1, TLV_DB_SCALE_ITEM(-2056, 250, 0),
	2, 3, TLV_DB_SCALE_ITEM(-1454, 250, 0),
	4, 5, TLV_DB_SCALE_ITEM(-928, 326, 0),
	6, 7, TLV_DB_SCALE_ITEM(-287, 287, 0)
);

static const struct snd_kcontrol_new am_controls[] = {
	SOC_SINGLE_TLV("AP Playback Volume", AM_DMIX1, AM_DMIX1_LVL1_SHIFT,
		       7, 1, am_mix_tlv),
};

static const struct snd_kcontrol_new am_dmix_controls[] = {
	SOC_DAPM_SINGLE("AP Switch", AM_DMIX1, AM_DMIX1_EN1_SHIFT, 1, 0),
};

static const char * const am_ap_capture_texts[] = {
	"Mixer", "Codec", "Modem Mixer",
};

static SOC_ENUM_SINGLE_DECL(am_ap_capture_enum, AM_DOUTMX1,
			    AM_DOUTMX1_SEL1_SHIFT, am_ap_capture_texts);

static const struct snd_kcontrol_new am_ap_capture_mux =
	SOC_DAPM_ENUM("AP Capture Source", am_ap_capture_enum);

static const struct snd_soc_dapm_widget am_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Mixer", AM_DIG_EN, AM_DIG_EN_MIX_SHIFT, 0,
			    NULL, 0),

	SND_SOC_DAPM_AIF_IN("AP RX", "AP Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AP TX", "AP Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("CODEC TX", "CODEC Capture", 0, SND_SOC_NOPM,
			     0, 0),
	SND_SOC_DAPM_AIF_IN("CODEC RX", "CODEC Playback", 0, SND_SOC_NOPM,
			    0, 0),

	SND_SOC_DAPM_MIXER("DMIX", SND_SOC_NOPM, 0, 0, am_dmix_controls,
			   ARRAY_SIZE(am_dmix_controls)),
	SND_SOC_DAPM_MUX("AP Capture Source", SND_SOC_NOPM, 0, 0,
			 &am_ap_capture_mux),
};

static const struct snd_soc_dapm_route am_routes[] = {
	{ "DMIX", "AP Switch", "AP RX" },
	{ "DMIX", NULL, "Mixer" },
	{ "CODEC TX", NULL, "DMIX" },

	{ "AP Capture Source", "Mixer", "DMIX" },
	{ "AP Capture Source", "Codec", "CODEC RX" },
	{ "AP Capture Source", NULL, "Mixer" },
	{ "AP TX", NULL, "AP Capture Source" },
};

static int am_startup(struct snd_pcm_substream *substream,
		      struct snd_soc_dai *dai)
{
	struct exynos7870_audmixer *am = snd_soc_dai_get_drvdata(dai);

	return pm_runtime_resume_and_get(am->dev);
}

static void am_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct exynos7870_audmixer *am = snd_soc_dai_get_drvdata(dai);

	pm_runtime_put_autosuspend(am->dev);
}

static int am_set_sysclk_rate(struct exynos7870_audmixer *am,
			      unsigned int rate)
{
	unsigned long target = rate * AM_SYSCLK_FS;
	unsigned long tol = target / 1000;
	unsigned long got;
	int ret;

	/*
	 * AUD_PLL runs a few Hz above the nominal 98.304 MHz, so ask for a
	 * little more than the nominal rate to get the expected divider.
	 */
	ret = clk_set_rate(am->sysclk, target + tol);
	if (ret)
		return ret;

	got = clk_get_rate(am->sysclk);
	if (got < target - tol || got > target + tol) {
		dev_err(am->dev, "sysclk is %lu Hz, wanted %lu Hz\n", got,
			target);
		return -EINVAL;
	}

	return 0;
}

static int am_ap_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct exynos7870_audmixer *am = snd_soc_dai_get_drvdata(dai);
	unsigned int bfs, xfs, dl;
	int ret;

	switch (params_width(params)) {
	case 16:
		dl = AM_DL_16BIT;
		break;
	case 24:
		dl = AM_DL_24BIT;
		break;
	default:
		return -EINVAL;
	}

	/* Without a ratio from the card, assume the I2S default of 2 slots */
	bfs = am->bclk_ratio ?: 2 * params_physical_width(params);
	switch (bfs) {
	case 32:
		xfs = AM_XFS_32FS;
		break;
	case 48:
		xfs = AM_XFS_48FS;
		break;
	case 64:
		xfs = AM_XFS_64FS;
		break;
	default:
		dev_err(am->dev, "unsupported bclk ratio %u\n", bfs);
		return -EINVAL;
	}

	if (params_rate(params) != 48000)
		return -EINVAL;

	ret = am_set_sysclk_rate(am, params_rate(params));
	if (ret)
		return ret;

	ret = regmap_update_bits(am->regmap, AM_IN1_CTL2,
				 AM_INCTL2_XFS | AM_INCTL2_DL,
				 FIELD_PREP(AM_INCTL2_XFS, xfs) |
				 FIELD_PREP(AM_INCTL2_DL, dl));
	if (ret)
		return ret;

	ret = regmap_clear_bits(am->regmap, AM_HQ_CTL, AM_HQ_CTL_HQ_EN);
	if (ret)
		return ret;

	/* Flush the data path after a configuration change */
	return am_reset_data(am);
}

static int am_ap_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct exynos7870_audmixer *am = snd_soc_dai_get_drvdata(dai);
	unsigned int ctl2 = 0;

	/* The SoC I2S controller is the bus master of the AP input */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctl2 |= FIELD_PREP(AM_INCTL2_DF, 1);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctl2 |= FIELD_PREP(AM_INCTL2_DF, 2);
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		ctl2 |= AM_INCTL2_LRCK_POL;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(am->regmap, AM_IN1_CTL2,
				  AM_INCTL2_DF | AM_INCTL2_LRCK_POL, ctl2);
}

static int am_ap_set_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio)
{
	struct exynos7870_audmixer *am = snd_soc_dai_get_drvdata(dai);

	am->bclk_ratio = ratio;

	return 0;
}

static int am_codec_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	/* The codec bus has a fixed format; only 48 kHz is supported yet */
	return params_rate(params) == 48000 ? 0 : -EINVAL;
}

static int am_codec_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* The mixer is the I2S bus master towards the codec */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BP_FP ||
	    (fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	return 0;
}

static const struct snd_soc_dai_ops am_ap_dai_ops = {
	.startup = am_startup,
	.shutdown = am_shutdown,
	.hw_params = am_ap_hw_params,
	.set_fmt = am_ap_set_fmt,
	.set_bclk_ratio = am_ap_set_bclk_ratio,
};

static const struct snd_soc_dai_ops am_codec_dai_ops = {
	.startup = am_startup,
	.shutdown = am_shutdown,
	.hw_params = am_codec_hw_params,
	.set_fmt = am_codec_set_fmt,
};

static struct snd_soc_dai_driver am_dais[] = {
	[AM_DAI_AP] = {
		.name = "audmixer-ap",
		.id = AM_DAI_AP,
		.playback = {
			.stream_name = "AP Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
		},
		.capture = {
			.stream_name = "AP Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
		},
		.ops = &am_ap_dai_ops,
		.symmetric_rate = 1,
		.symmetric_sample_bits = 1,
	},
	[AM_DAI_CODEC] = {
		/*
		 * In a codec-to-codec link the capture stream of this DAI
		 * carries the data to the codec.
		 */
		.name = "audmixer-codec",
		.id = AM_DAI_CODEC,
		.playback = {
			.stream_name = "CODEC Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.capture = {
			.stream_name = "CODEC Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &am_codec_dai_ops,
		.symmetric_rate = 1,
	},
};

static const struct snd_soc_component_driver am_component = {
	.name = "exynos7870-audmixer",
	.controls = am_controls,
	.num_controls = ARRAY_SIZE(am_controls),
	.dapm_widgets = am_widgets,
	.num_dapm_widgets = ARRAY_SIZE(am_widgets),
	.dapm_routes = am_routes,
	.num_dapm_routes = ARRAY_SIZE(am_routes),
	.use_pmdown_time = 1,
	.endianness = 1,
};

/* Codec master clock on the MCKO pin; it runs from the mixer system clock */
static int am_mclk_prepare(struct clk_hw *hw)
{
	struct exynos7870_audmixer *am =
		container_of(hw, struct exynos7870_audmixer, mclk);
	int ret;

	ret = pm_runtime_resume_and_get(am->dev);
	if (ret)
		return ret;

	ret = regmap_set_bits(am->regmap, AM_HQ_CTL, AM_HQ_CTL_MCKO_EN);
	if (ret)
		pm_runtime_put_autosuspend(am->dev);

	return ret;
}

static void am_mclk_unprepare(struct clk_hw *hw)
{
	struct exynos7870_audmixer *am =
		container_of(hw, struct exynos7870_audmixer, mclk);

	regmap_clear_bits(am->regmap, AM_HQ_CTL, AM_HQ_CTL_MCKO_EN);
	pm_runtime_put_autosuspend(am->dev);
}

static const struct clk_ops am_mclk_ops = {
	.prepare = am_mclk_prepare,
	.unprepare = am_mclk_unprepare,
};

static int am_register_mclk(struct exynos7870_audmixer *am)
{
	struct device *dev = am->dev;
	struct clk_parent_data parent = { .fw_name = "sysclk" };
	struct clk_init_data init = {
		.ops = &am_mclk_ops,
		.parent_data = &parent,
		.num_parents = 1,
	};
	int ret;

	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s_mclk", dev_name(dev));
	if (!init.name)
		return -ENOMEM;
	of_property_read_string(dev->of_node, "clock-output-names", &init.name);

	am->mclk.init = &init;
	ret = devm_clk_hw_register(dev, &am->mclk);
	if (ret)
		return ret;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					   &am->mclk);
}

static int am_runtime_suspend(struct device *dev)
{
	struct exynos7870_audmixer *am = dev_get_drvdata(dev);

	regcache_cache_only(am->regmap, true);
	regcache_mark_dirty(am->regmap);
	pinctrl_pm_select_sleep_state(dev);
	clk_disable_unprepare(am->sysclk);
	clk_disable_unprepare(am->pclk);

	return regmap_set_bits(am->pmu, EXYNOS7870_PMU_AUD_PATH_CFG,
			       AUD_PATH_CFG_MIXER_OFF);
}

static int am_runtime_resume(struct device *dev)
{
	struct exynos7870_audmixer *am = dev_get_drvdata(dev);
	int ret;

	ret = regmap_clear_bits(am->pmu, EXYNOS7870_PMU_AUD_PATH_CFG,
				AUD_PATH_CFG_MIXER_OFF);
	if (ret)
		return ret;

	ret = clk_prepare_enable(am->pclk);
	if (ret)
		goto err_pmu;

	ret = clk_prepare_enable(am->sysclk);
	if (ret)
		goto err_pclk;

	pinctrl_pm_select_default_state(dev);
	regcache_cache_only(am->regmap, false);

	/* Reset the mixer logic, then restore its configuration */
	ret = regmap_write(am->regmap, AM_SOFT_RSTB, 0);
	if (ret)
		goto err_clk;
	usleep_range(1000, 1100);
	ret = regmap_write(am->regmap, AM_SOFT_RSTB,
			   AM_SOFT_RSTB_SYS | AM_SOFT_RSTB_DATA);
	if (ret)
		goto err_clk;
	usleep_range(1000, 1100);

	ret = regcache_sync(am->regmap);
	if (ret)
		goto err_clk;

	return 0;

err_clk:
	regcache_cache_only(am->regmap, true);
	pinctrl_pm_select_sleep_state(dev);
	clk_disable_unprepare(am->sysclk);
err_pclk:
	clk_disable_unprepare(am->pclk);
err_pmu:
	regmap_set_bits(am->pmu, EXYNOS7870_PMU_AUD_PATH_CFG,
			AUD_PATH_CFG_MIXER_OFF);
	return ret;
}

static int am_resume(struct device *dev)
{
	struct exynos7870_audmixer *am = dev_get_drvdata(dev);

	/*
	 * The block reset is outside the mixer and may have been lost with
	 * the power domain; pulse it like at probe.
	 */
	reset_control_reset(am->rst);

	return pm_runtime_force_resume(dev);
}

static int am_reset_block(struct exynos7870_audmixer *am)
{
	/* Pulse the reset when the controller can, else only release it */
	if (!reset_control_reset(am->rst))
		return 0;

	return reset_control_deassert(am->rst);
}

static int am_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7870_audmixer *am;
	void __iomem *regs;
	unsigned int reg, val;
	int ret;

	am = devm_kzalloc(dev, sizeof(*am), GFP_KERNEL);
	if (!am)
		return -ENOMEM;

	am->dev = dev;
	platform_set_drvdata(pdev, am);

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	am->regmap = devm_regmap_init_mmio(dev, regs, &am_regmap_config);
	if (IS_ERR(am->regmap))
		return dev_err_probe(dev, PTR_ERR(am->regmap),
				     "Failed to init regmap\n");

	am->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						  "samsung,pmu-syscon");
	if (IS_ERR(am->pmu))
		return dev_err_probe(dev, PTR_ERR(am->pmu),
				     "Failed to get PMU regmap\n");

	am->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(am->pclk))
		return dev_err_probe(dev, PTR_ERR(am->pclk),
				     "Failed to get pclk\n");

	am->sysclk = devm_clk_get(dev, "sysclk");
	if (IS_ERR(am->sysclk))
		return dev_err_probe(dev, PTR_ERR(am->sysclk),
				     "Failed to get sysclk\n");

	am->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(am->rst))
		return dev_err_probe(dev, PTR_ERR(am->rst),
				     "Failed to get reset\n");

	ret = am_reset_block(am);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to release reset\n");

	/* The sysclk divider is left at 2 by the bootloader */
	ret = am_set_sysclk_rate(am, 48000);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set sysclk rate\n");

	regcache_cache_only(am->regmap, true);

	pm_runtime_set_autosuspend_delay(dev, AM_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	/* Fill the cache so that controls can be read while suspended */
	for (reg = 0; reg <= AM_ALC_SGR; reg += 4) {
		if (!am_accessible_reg(dev, reg) || am_volatile_reg(dev, reg))
			continue;
		ret = regmap_read(am->regmap, reg, &val);
		if (ret)
			goto err_put;
	}

	ret = regmap_multi_reg_write(am->regmap, am_init_seq,
				     ARRAY_SIZE(am_init_seq));
	if (ret)
		goto err_put;

	ret = am_reset_data(am);
	if (ret)
		goto err_put;

	ret = am_register_mclk(am);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register MCLK\n");
		goto err_put;
	}

	ret = devm_snd_soc_register_component(dev, &am_component, am_dais,
					      ARRAY_SIZE(am_dais));
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register component\n");
		goto err_put;
	}

	pm_runtime_put_autosuspend(dev);

	return 0;

err_put:
	pm_runtime_put_sync(dev);
	return ret;
}

static const struct dev_pm_ops am_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, am_resume)
	RUNTIME_PM_OPS(am_runtime_suspend, am_runtime_resume, NULL)
};

static const struct of_device_id am_of_match[] = {
	{ .compatible = "samsung,exynos7870-audmixer" },
	{ }
};
MODULE_DEVICE_TABLE(of, am_of_match);

static struct platform_driver am_driver = {
	.driver = {
		.name = "exynos7870-audmixer",
		.of_match_table = am_of_match,
		.pm = pm_ptr(&am_pm_ops),
	},
	.probe = am_probe,
};
module_platform_driver(am_driver);

MODULE_DESCRIPTION("Samsung Exynos7870 audio mixer driver");
MODULE_LICENSE("GPL");
