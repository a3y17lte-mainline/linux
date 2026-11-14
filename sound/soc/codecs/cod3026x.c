// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung COD3026X audio codec
 *
 * The codec is packaged with the S2MPU05 PMIC of Exynos7870 phones and gets
 * its master clock from the SoC audio mixer, which is also the bus master
 * of its I2S interface (64 fs, 24-bit slots).
 *
 * Without the master clock only the detection block (0x80-0x8b) and the
 * interrupt registers respond; the rest of the register file reads zero
 * and loses its contents, so it is cached and written back each time the
 * clock comes back. The trim registers 0xd0-0xde are loaded by the chip at
 * power-up; they are saved once and written back as well, as some of them
 * are changed depending on the active outputs.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define COD_IRQ1PEND		0x01
#define COD_IRQ2PEND		0x02
#define COD_IRQ3PEND		0x03
#define  COD_IRQ_JACK_DET	BIT(0)
#define COD_IRQ5PEND		0x05
#define COD_IRQ1M		0x06
#define COD_IRQ2M		0x07
#define COD_IRQ3M		0x08
#define COD_IRQ5M		0x0a
#define COD_STATUS1		0x0b
#define  COD_STATUS1_JACK	BIT(0)
#define COD_STATUS3		0x0d
#define COD_PD_REF		0x10
#define  COD_PD_REF_VMID	BIT(5)
#define  COD_PD_REF_IGEN	BIT(4)
#define  COD_PD_REF_IGEN_AD	BIT(3)
#define  COD_PD_REF_MCB1_SHIFT	2
#define  COD_PD_REF_MCB2_SHIFT	1
#define  COD_PD_REF_MCB_LDO_SHIFT 0
#define COD_PD_AD1		0x11
#define  COD_PD_AD1_PDB_MIXR	BIT(7)
#define  COD_PD_AD1_PDB_MIXL	BIT(6)
#define  COD_PD_AD1_DSML_PREQ	BIT(5)
#define  COD_PD_AD1_DSMR_PREQ	BIT(4)
#define  COD_PD_AD1_PDB_DSML	BIT(3)
#define  COD_PD_AD1_PDB_DSMR	BIT(2)
#define  COD_PD_AD1_RESETB_DSML	BIT(1)
#define  COD_PD_AD1_RESETB_DSMR	BIT(0)
#define COD_PD_AD2		0x12
#define  COD_PD_AD2_MIC_BST(n)	BIT(6 - (n))	/* n = 1..3 */
#define  COD_PD_AD2_MIC_PGA(n)	BIT(3 - (n))
#define COD_PWAUTO_DA		0x17
#define  COD_PWAUTO_DA_AUTO	BIT(4)
#define  COD_PWAUTO_DA_SPK	BIT(2)
#define  COD_PWAUTO_DA_HP	BIT(1)
#define  COD_PWAUTO_DA_EP	BIT(0)
#define COD_CTRL_REF		0x18
#define  COD_CTRL_REF_VMID	GENMASK(7, 6)
#define  COD_CTRL_REF_VMID_50K	1
#define  COD_CTRL_REF_VMID_5K	0
#define  COD_CTRL_REF_MCB1	GENMASK(5, 4)
#define  COD_CTRL_REF_MCB2_EN	BIT(3)
#define COD_SV_HP		0x19
#define  COD_SV_HP_SKIP		BIT(5)
#define COD_VOL_AD1		0x20
#define COD_VOL_AD2		0x21
#define COD_VOL_AD3		0x22
#define  COD_VOL_AD_BST_SHIFT	5
#define  COD_VOL_AD_PGA_SHIFT	0
#define COD_MIX_AD1		0x23
#define  COD_MIX_AD1_MICL(n)	BIT(7 - 2 * (n))	/* n = 1..3 */
#define  COD_MIX_AD1_MICR(n)	BIT(6 - 2 * (n))
#define COD_MIX_AD2		0x24
#define COD_DSM_ADS		0x26
#define COD_VOL_HPL		0x30
#define COD_VOL_HPR		0x31
#define COD_VOL_EP_SPK		0x32
#define  COD_VOL_EP_SHIFT	4
#define COD_CTRL_EP		0x33
#define  COD_CTRL_EP_CP_MODE	GENMASK(5, 4)
#define COD_DCT_CLK1		0x38
#define COD_MIX_DA1		0x36
#define  COD_MIX_DA1_HPL_DCTL	BIT(7)
#define  COD_MIX_DA1_HPR_DCTR	BIT(2)
#define COD_MIX_DA2		0x37
#define  COD_MIX_DA2_EP_DCTL	BIT(7)
#define COD_DIGITAL_POWER	0x40
#define  COD_DP_PDB_ADCDIG	BIT(7)
#define  COD_DP_RSTB_DAT_AD	BIT(6)
#define  COD_DP_PDB_DACDIG	BIT(5)
#define  COD_DP_RSTB_DAT_DA	BIT(4)
#define COD_FORMAT		0x41
#define  COD_FORMAT_WORD_LEN	GENMASK(7, 6)
#define  COD_FORMAT_WORD_LEN_24	2
#define  COD_FORMAT_I2S		BIT(3)
#define  COD_FORMAT_LRJ		BIT(2)
#define  COD_FORMAT_BCLK_POL	BIT(1)
#define  COD_FORMAT_LRCLK_POL	BIT(0)
#define COD_ADC1		0x42
#define  COD_ADC1_MUTE		BIT(0)
#define COD_ADC_L_VOL		0x43
#define COD_ADC_R_VOL		0x44
#define COD_DAC1		0x50
#define  COD_DAC1_MONOMIX_SHIFT	4
#define COD_DAC_L_VOL		0x51
#define COD_DAC_R_VOL		0x52
#define COD_DNC1		0x54
#define  COD_DNC1_EN		BIT(7)
#define  COD_DNC1_START_GAIN	BIT(6)
#define COD_DNC2		0x55
#define  COD_DNC2_MAX_GAIN	GENMASK(4, 0)
#define COD_DNC4		0x57
#define  COD_DNC4_WINSEL	GENMASK(7, 5)
#define  COD_DNC4_WINSEL_20HZ	7
#define COD_DNC7		0x5a
#define COD_DNC9		0x5c
#define COD_SPKLIMIT3		0x5f
#define COD_OFFSET1		0x60
#define COD_RESERVED		0x61
#define COD_IRQ_R		0x62
#define COD_CLK1_AD		0x70
#define  COD_CLK1_AD_CHCLK	GENMASK(5, 4)
#define COD_AD_MIC3N_TRIM1	0x75
#define COD_CHOP_DA		0x76
#define  COD_CHOP_DA_HP_SHIFT	4
#define  COD_CHOP_DA_EP_SHIFT	3
#define  COD_CHOP_DA_HP		BIT(COD_CHOP_DA_HP_SHIFT)
#define  COD_CHOP_DA_EP		BIT(COD_CHOP_DA_EP_SHIFT)
#define  COD_CHOP_DA_SPK	BIT(2)
#define COD_CTRL_CP		0x77
#define COD_MIC_ON		0x78	/* also the MIC3N trim, bit 0 */
#define  COD_MIC_ON_MIC(n)	BIT(3 + (n))	/* n = 1..3 */
#define  COD_MIC_ON_ALL		GENMASK(7, 4)
#define COD_SL_DA2		0x7a
#define COD_DET_PDB		0x80
#define COD_DET_ON		0x81
#define  COD_DET_ON_PDB_JD_CLK	BIT(1)
#define COD_MIC_BIAS		0x82
#define  COD_MIC_BIAS_MCB2	GENMASK(3, 2)
#define  COD_MIC_BIAS_LDO	GENMASK(1, 0)
#define COD_JACK_DET2		0x84
#define COD_MIC_DET		0x85
#define COD_DET_TIME		0x86
#define COD_LDO_DIG		0x87
#define COD_KEY_TIME		0x88
#define COD_MICBIAS2		0x8b
#define COD_TRIM_FIRST		0xd0
#define COD_OFFSET_DAL		0xd4
#define COD_OFFSET_DAR		0xd5
#define COD_CTRL_EP_TRIM	0xd7
#define COD_TRIM_LAST		0xde
#define COD_NUM_TRIMS		(COD_TRIM_LAST - COD_TRIM_FIRST + 1)

#define COD_ADC_UNMUTE_DELAY_MS	220
#define COD_JACK_BUTTONS	4
#define COD_ADC_SAMPLES		5
#define COD_BUTTON_DELAY_MS	30
#define COD_JACK_DETECT_DELAY_MS 550

struct cod3026x {
	struct device *dev;
	struct regmap *regmap;
	struct regulator *vdd;
	struct clk *mclk;
	struct delayed_work adc_unmute_work;
	u8 trims[COD_NUM_TRIMS];
	bool trims_saved;
	u8 mcb1, mcb2, mcb_ldo;
	unsigned int hp_vol[2];

	/* Jack detection, on the always-on part of the chip */
	struct regmap *det_regmap;
	struct iio_channel *jack_adc;
	struct snd_soc_component *component;
	struct snd_soc_jack jack;
	struct delayed_work jack_work;
	struct delayed_work button_work;
	struct mutex jack_lock;		/* jack and button state */
	int irq;
	bool jack_in;
	int jack_type;
	int button;
	u32 mic_uv;
	u32 release_uv;
	u32 button_uv[COD_JACK_BUTTONS];
};

static bool cod_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case COD_IRQ1PEND ... COD_STATUS3:
	case COD_PD_REF ... 0x1c:
	case COD_VOL_AD1 ... COD_DSM_ADS:
	case COD_VOL_HPL ... COD_DCT_CLK1:
	case COD_DIGITAL_POWER ... COD_ADC_R_VOL:
	case COD_DAC1 ... COD_SPKLIMIT3:
	case COD_OFFSET1 ... COD_IRQ_R:
	case COD_CLK1_AD ... COD_SL_DA2:
	case COD_DET_PDB ... COD_MICBIAS2:
	case COD_TRIM_FIRST ... COD_TRIM_LAST:
		return true;
	default:
		return false;
	}
}

static bool cod_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case COD_STATUS1 ... COD_STATUS3:
	case COD_RESERVED ... COD_IRQ_R:
		return false;
	default:
		return cod_readable_reg(dev, reg);
	}
}

/*
 * The detection block keeps its state without the master clock and is
 * accessed directly, like the interrupt and status registers. The trims
 * are handled separately.
 */
static bool cod_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case COD_IRQ1PEND ... COD_STATUS3:
	case COD_RESERVED ... COD_IRQ_R:
	case COD_DET_PDB ... COD_MICBIAS2:
	case COD_TRIM_FIRST ... COD_TRIM_LAST:
		return true;
	default:
		return false;
	}
}

/*
 * The interrupt, status and detection registers keep working without the
 * master clock, while the main register map is cache-only: give them an
 * uncached map of their own.
 */
static bool cod_det_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case COD_IRQ1PEND ... COD_STATUS3:
	case COD_DET_PDB ... COD_MICBIAS2:
		return true;
	default:
		return false;
	}
}

static bool cod_det_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case COD_IRQ1M ... COD_IRQ5M:
	case COD_DET_PDB ... COD_MICBIAS2:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config cod_det_regmap_config = {
	.name = "det",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = COD_MICBIAS2,
	.readable_reg = cod_det_readable_reg,
	.writeable_reg = cod_det_writeable_reg,
	.use_single_read = true,
	.use_single_write = true,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config cod_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = COD_TRIM_LAST,
	.readable_reg = cod_readable_reg,
	.writeable_reg = cod_writeable_reg,
	.volatile_reg = cod_volatile_reg,
	.use_single_read = true,
	.use_single_write = true,
	.cache_type = REGCACHE_MAPLE,
};

/* 0, 12, 20 dB */
static const DECLARE_TLV_DB_RANGE(cod_mic_boost_tlv,
	0, 1, TLV_DB_SCALE_ITEM(0, 1200, 0),
	2, 2, TLV_DB_SCALE_ITEM(2000, 0, 0)
);

static const DECLARE_TLV_DB_SCALE(cod_mic_pga_tlv, -1650, 150, 0);
static const DECLARE_TLV_DB_SCALE(cod_hp_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(cod_ep_tlv, 0, 100, 0);

/*
 * Digital volumes, register 0x00 = +12 dB descending: 0.5 dB steps down to
 * -55 dB (0x86), then 1 dB steps to -66 dB (0x91), 2 dB steps to -72 dB
 * (0x94), then -78 and -84 dB. Controls are inverted.
 */
#define COD_DVOL_MAX		0x96
static const DECLARE_TLV_DB_RANGE(cod_dvol_tlv,
	0, 1, TLV_DB_SCALE_ITEM(-8400, 600, 0),
	2, 4, TLV_DB_SCALE_ITEM(-7200, 200, 0),
	5, 15, TLV_DB_SCALE_ITEM(-6600, 100, 0),
	16, 150, TLV_DB_SCALE_ITEM(-5500, 50, 0)
);

static const char * const cod_mono_mix_text[] = {
	"Disable", "Right", "Left", "Left Right Invert", "(L+R)/2", "L+R",
};

static SOC_ENUM_SINGLE_DECL(cod_mono_mix_enum, COD_DAC1,
			    COD_DAC1_MONOMIX_SHIFT, cod_mono_mix_text);

static const struct snd_kcontrol_new cod_controls[] = {
	SOC_SINGLE_TLV("MIC1 Boost Volume", COD_VOL_AD1, COD_VOL_AD_BST_SHIFT,
		       2, 0, cod_mic_boost_tlv),
	SOC_SINGLE_TLV("MIC1 Volume", COD_VOL_AD1, COD_VOL_AD_PGA_SHIFT,
		       31, 1, cod_mic_pga_tlv),
	SOC_SINGLE_TLV("MIC2 Boost Volume", COD_VOL_AD2, COD_VOL_AD_BST_SHIFT,
		       2, 0, cod_mic_boost_tlv),
	SOC_SINGLE_TLV("MIC2 Volume", COD_VOL_AD2, COD_VOL_AD_PGA_SHIFT,
		       31, 1, cod_mic_pga_tlv),
	SOC_SINGLE_TLV("MIC3 Boost Volume", COD_VOL_AD3, COD_VOL_AD_BST_SHIFT,
		       2, 0, cod_mic_boost_tlv),
	SOC_SINGLE_TLV("MIC3 Volume", COD_VOL_AD3, COD_VOL_AD_PGA_SHIFT,
		       31, 1, cod_mic_pga_tlv),

	SOC_DOUBLE_R_TLV("Headphone Playback Volume", COD_VOL_HPL, COD_VOL_HPR,
			 0, 63, 1, cod_hp_tlv),
	SOC_SINGLE_TLV("Earpiece Playback Volume", COD_VOL_EP_SPK,
		       COD_VOL_EP_SHIFT, 12, 0, cod_ep_tlv),

	SOC_DOUBLE_R_TLV("DAC Playback Volume", COD_DAC_L_VOL, COD_DAC_R_VOL,
			 0, COD_DVOL_MAX, 1, cod_dvol_tlv),
	SOC_DOUBLE_R_TLV("ADC Capture Volume", COD_ADC_L_VOL, COD_ADC_R_VOL,
			 0, COD_DVOL_MAX, 1, cod_dvol_tlv),

	SOC_ENUM("Mono Mix", cod_mono_mix_enum),
};

static unsigned int cod_active_outputs(struct snd_soc_component *component)
{
	return snd_soc_component_read(component, COD_CHOP_DA) &
	       (COD_CHOP_DA_HP | COD_CHOP_DA_EP | COD_CHOP_DA_SPK);
}

/*
 * The DAC offset trims depend on the active outputs: the factory values
 * for the headphones alone, the earpiece trim for the earpiece alone, and
 * zero (with the dynamic noise control off) for any combination.
 */
static void cod_update_dac_offset(struct snd_soc_component *component)
{
	struct cod3026x *cod = snd_soc_component_get_drvdata(component);
	unsigned int outputs = cod_active_outputs(component);
	unsigned int ep_trim;

	if (!outputs || !cod->trims_saved)
		return;

	if (outputs == COD_CHOP_DA_HP) {
		regmap_write(cod->regmap, COD_OFFSET_DAL,
			     cod->trims[COD_OFFSET_DAL - COD_TRIM_FIRST]);
		regmap_write(cod->regmap, COD_OFFSET_DAR,
			     cod->trims[COD_OFFSET_DAR - COD_TRIM_FIRST]);
		return;
	}

	if (outputs == COD_CHOP_DA_EP) {
		if (!regmap_read(cod->regmap, COD_CTRL_EP_TRIM, &ep_trim))
			regmap_write(cod->regmap, COD_OFFSET_DAL, ep_trim);
	} else {
		regmap_write(cod->regmap, COD_OFFSET_DAL, 0);
		regmap_write(cod->regmap, COD_OFFSET_DAR, 0);
	}

	snd_soc_component_update_bits(component, COD_DNC1, COD_DNC1_EN, 0);
	usleep_range(100, 200);
}

static int cod_power_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3026x *cod = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return pm_runtime_resume_and_get(cod->dev);
	case SND_SOC_DAPM_POST_PMD:
		pm_runtime_put_autosuspend(cod->dev);
		break;
	}

	return 0;
}

static int cod_dac_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_PDB_DACDIG,
					      COD_DP_PDB_DACDIG);
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_RSTB_DAT_DA, 0);
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_RSTB_DAT_DA,
					      COD_DP_RSTB_DAT_DA);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_RSTB_DAT_DA, 0);
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_PDB_DACDIG, 0);
		break;
	}

	return 0;
}

static int cod_ep_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	unsigned int others;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod_update_dac_offset(component);
		snd_soc_component_update_bits(component, COD_PWAUTO_DA,
					      COD_PWAUTO_DA_AUTO | COD_PWAUTO_DA_EP,
					      COD_PWAUTO_DA_AUTO | COD_PWAUTO_DA_EP);
		snd_soc_component_update_bits(component, COD_MIX_DA2,
					      COD_MIX_DA2_EP_DCTL,
					      COD_MIX_DA2_EP_DCTL);
		msleep(136);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		others = cod_active_outputs(component) & ~COD_CHOP_DA_EP;
		snd_soc_component_update_bits(component, COD_PWAUTO_DA,
					      COD_PWAUTO_DA_EP |
					      (others ? 0 : COD_PWAUTO_DA_AUTO), 0);
		usleep_range(100, 200);
		snd_soc_component_update_bits(component, COD_MIX_DA2,
					      COD_MIX_DA2_EP_DCTL, 0);
		usleep_range(100, 200);
		break;
	}

	return 0;
}

static void cod_hp_set_volume(struct snd_soc_component *component,
			      unsigned int left, unsigned int right)
{
	snd_soc_component_write(component, COD_VOL_HPL, left);
	snd_soc_component_write(component, COD_VOL_HPR, right);
}

static int cod_hp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3026x *cod = snd_soc_component_get_drvdata(component);
	unsigned int others = cod_active_outputs(component) & ~COD_CHOP_DA_HP;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod->hp_vol[0] = snd_soc_component_read(component, COD_VOL_HPL);
		cod->hp_vol[1] = snd_soc_component_read(component, COD_VOL_HPR);

		/* Prime the dynamic noise control with its start gain */
		snd_soc_component_update_bits(component, COD_DNC1,
					      COD_DNC1_START_GAIN,
					      COD_DNC1_START_GAIN);
		snd_soc_component_write(component, COD_DNC7, 0x1a);
		snd_soc_component_update_bits(component, COD_DNC1, COD_DNC1_EN,
					      COD_DNC1_EN);
		usleep_range(100, 200);
		snd_soc_component_update_bits(component, COD_DNC1, COD_DNC1_EN,
					      0);

		cod_hp_set_volume(component, 0x1a, 0x1a);
		snd_soc_component_update_bits(component, COD_SV_HP,
					      COD_SV_HP_SKIP, COD_SV_HP_SKIP);
		cod_update_dac_offset(component);
		snd_soc_component_update_bits(component, COD_DNC4,
					      COD_DNC4_WINSEL,
					      FIELD_PREP(COD_DNC4_WINSEL,
							 COD_DNC4_WINSEL_20HZ));
		usleep_range(100, 200);
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, COD_PWAUTO_DA,
					      COD_PWAUTO_DA_AUTO | COD_PWAUTO_DA_HP,
					      COD_PWAUTO_DA_AUTO | COD_PWAUTO_DA_HP);
		snd_soc_component_update_bits(component, COD_MIX_DA1,
					      COD_MIX_DA1_HPL_DCTL |
					      COD_MIX_DA1_HPR_DCTR,
					      COD_MIX_DA1_HPL_DCTL |
					      COD_MIX_DA1_HPR_DCTR);
		msleep(180);
		snd_soc_component_update_bits(component, COD_SV_HP,
					      COD_SV_HP_SKIP, 0);

		if (!others) {
			/* Headphones alone: the noise control sets the gain */
			cod_hp_set_volume(component, 0x1a, 0x1a);
			usleep_range(6000, 7000);
			snd_soc_component_update_bits(component, COD_DNC1,
						      COD_DNC1_EN, COD_DNC1_EN);
		} else {
			cod_hp_set_volume(component, cod->hp_vol[0],
					  cod->hp_vol[1]);
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, COD_DNC1, COD_DNC1_EN,
					      0);
		snd_soc_component_update_bits(component, COD_SV_HP,
					      COD_SV_HP_SKIP, COD_SV_HP_SKIP);
		snd_soc_component_update_bits(component, COD_PWAUTO_DA,
					      COD_PWAUTO_DA_HP |
					      (others ? 0 : COD_PWAUTO_DA_AUTO), 0);
		snd_soc_component_update_bits(component, COD_MIX_DA1,
					      COD_MIX_DA1_HPL_DCTL |
					      COD_MIX_DA1_HPR_DCTR, 0);
		msleep(40);
		snd_soc_component_update_bits(component, COD_SV_HP,
					      COD_SV_HP_SKIP, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Give the user volume back to the control */
		cod_hp_set_volume(component, cod->hp_vol[0], cod->hp_vol[1]);
		break;
	}

	return 0;
}

static void cod_adc_mute(struct snd_soc_component *component, bool mute)
{
	snd_soc_component_update_bits(component, COD_ADC1, COD_ADC1_MUTE,
				      mute ? COD_ADC1_MUTE : 0);
}

static void cod_adc_unmute_work(struct work_struct *work)
{
	struct cod3026x *cod = container_of(work, struct cod3026x,
					    adc_unmute_work.work);

	regmap_update_bits(cod->regmap, COD_ADC1, COD_ADC1_MUTE, 0);
}

static int cod_adc_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3026x *cod = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Unmute once the analog path has settled */
		schedule_delayed_work(&cod->adc_unmute_work,
				      msecs_to_jiffies(COD_ADC_UNMUTE_DELAY_MS));
		break;
	case SND_SOC_DAPM_PRE_PMD:
		cancel_delayed_work_sync(&cod->adc_unmute_work);
		cod_adc_mute(component, true);
		break;
	}

	return 0;
}

/* Reference and ADC digital power for capture */
static int cod_vmid_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod_adc_mute(component, true);
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_PDB_ADCDIG,
					      COD_DP_PDB_ADCDIG);
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_RSTB_DAT_AD, 0);
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_RSTB_DAT_AD,
					      COD_DP_RSTB_DAT_AD);

		/* Charge the reference quickly through 5 kohm, then 50 kohm */
		snd_soc_component_update_bits(component, COD_PD_REF,
					      COD_PD_REF_VMID, COD_PD_REF_VMID);
		snd_soc_component_update_bits(component, COD_CTRL_REF,
					      COD_CTRL_REF_VMID,
					      FIELD_PREP(COD_CTRL_REF_VMID,
							 COD_CTRL_REF_VMID_5K));
		snd_soc_component_update_bits(component, COD_PD_AD1,
					      COD_PD_AD1_DSML_PREQ |
					      COD_PD_AD1_DSMR_PREQ,
					      COD_PD_AD1_DSML_PREQ |
					      COD_PD_AD1_DSMR_PREQ);
		msleep(140);
		snd_soc_component_update_bits(component, COD_CTRL_REF,
					      COD_CTRL_REF_VMID,
					      FIELD_PREP(COD_CTRL_REF_VMID,
							 COD_CTRL_REF_VMID_50K));
		snd_soc_component_update_bits(component, COD_PD_REF,
					      COD_PD_REF_IGEN, COD_PD_REF_IGEN);
		usleep_range(100, 200);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, COD_PD_REF,
					      COD_PD_REF_IGEN | COD_PD_REF_VMID,
					      0);
		snd_soc_component_update_bits(component, COD_DIGITAL_POWER,
					      COD_DP_RSTB_DAT_AD |
					      COD_DP_PDB_ADCDIG, 0);
		cod_adc_mute(component, false);
		break;
	}

	return 0;
}

static int cod_micbias2_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	snd_soc_component_update_bits(component, COD_CTRL_REF,
				      COD_CTRL_REF_MCB2_EN,
				      SND_SOC_DAPM_EVENT_ON(event) ?
				      COD_CTRL_REF_MCB2_EN : 0);

	return 0;
}

static void cod_mic_power_on(struct snd_soc_component *component, int n)
{
	unsigned int mix_mask = COD_MIX_AD1_MICL(n) | COD_MIX_AD1_MICR(n);
	unsigned int mix;

	/* Route both ADC channels when no route is selected */
	mix = snd_soc_component_read(component, COD_MIX_AD1) & mix_mask;
	if (!mix)
		mix = mix_mask;

	snd_soc_component_update_bits(component, COD_MIC_ON, COD_MIC_ON_MIC(n),
				      COD_MIC_ON_MIC(n));
	snd_soc_component_update_bits(component, COD_MIX_AD1, mix_mask, 0);
	snd_soc_component_update_bits(component, COD_PD_REF, COD_PD_REF_IGEN_AD,
				      COD_PD_REF_IGEN_AD);
	snd_soc_component_update_bits(component, COD_PD_AD2,
				      COD_PD_AD2_MIC_BST(n) |
				      COD_PD_AD2_MIC_PGA(n),
				      COD_PD_AD2_MIC_BST(n) |
				      COD_PD_AD2_MIC_PGA(n));
	snd_soc_component_update_bits(component, COD_PD_AD1,
				      COD_PD_AD1_PDB_MIXL | COD_PD_AD1_PDB_MIXR,
				      COD_PD_AD1_PDB_MIXL | COD_PD_AD1_PDB_MIXR);
	snd_soc_component_update_bits(component, COD_PD_AD1,
				      COD_PD_AD1_PDB_DSML | COD_PD_AD1_PDB_DSMR,
				      COD_PD_AD1_PDB_DSML | COD_PD_AD1_PDB_DSMR);
	snd_soc_component_update_bits(component, COD_MIX_AD1, mix_mask, mix);
	snd_soc_component_update_bits(component, COD_PD_AD1,
				      COD_PD_AD1_DSML_PREQ |
				      COD_PD_AD1_DSMR_PREQ, 0);
	snd_soc_component_update_bits(component, COD_PD_AD1,
				      COD_PD_AD1_RESETB_DSML |
				      COD_PD_AD1_RESETB_DSMR,
				      COD_PD_AD1_RESETB_DSML |
				      COD_PD_AD1_RESETB_DSMR);
}

static void cod_mic_power_off(struct snd_soc_component *component, int n)
{
	unsigned int others = snd_soc_component_read(component, COD_MIC_ON) &
			      COD_MIC_ON_ALL & ~COD_MIC_ON_MIC(n);

	if (!others)
		snd_soc_component_update_bits(component, COD_PD_AD1,
					      COD_PD_AD1_RESETB_DSML |
					      COD_PD_AD1_RESETB_DSMR, 0);
	if (!others) {
		snd_soc_component_update_bits(component, COD_PD_AD1,
					      COD_PD_AD1_PDB_DSML |
					      COD_PD_AD1_PDB_DSMR, 0);
		snd_soc_component_update_bits(component, COD_PD_AD1,
					      COD_PD_AD1_PDB_MIXL |
					      COD_PD_AD1_PDB_MIXR, 0);
	}
	snd_soc_component_update_bits(component, COD_PD_AD2,
				      COD_PD_AD2_MIC_BST(n) |
				      COD_PD_AD2_MIC_PGA(n), 0);
	if (!others)
		snd_soc_component_update_bits(component, COD_PD_REF,
					      COD_PD_REF_IGEN_AD, 0);
	snd_soc_component_update_bits(component, COD_MIC_ON, COD_MIC_ON_MIC(n),
				      0);
}

static int cod_mic_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod_mic_power_on(component, w->shift);
		break;
	case SND_SOC_DAPM_POST_PMD:
		cod_mic_power_off(component, w->shift);
		break;
	}

	return 0;
}

#define COD_MIC_PGA(wname, n) \
	SND_SOC_DAPM_PGA_E(wname, SND_SOC_NOPM, n, 0, NULL, 0, cod_mic_event, \
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD)

static const struct snd_kcontrol_new cod_adcl_mix[] = {
	SOC_DAPM_SINGLE("MIC1 Switch", COD_MIX_AD1, 5, 1, 0),
	SOC_DAPM_SINGLE("MIC2 Switch", COD_MIX_AD1, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC3 Switch", COD_MIX_AD1, 1, 1, 0),
};

static const struct snd_kcontrol_new cod_adcr_mix[] = {
	SOC_DAPM_SINGLE("MIC1 Switch", COD_MIX_AD1, 4, 1, 0),
	SOC_DAPM_SINGLE("MIC2 Switch", COD_MIX_AD1, 2, 1, 0),
	SOC_DAPM_SINGLE("MIC3 Switch", COD_MIX_AD1, 0, 1, 0),
};

static const struct snd_kcontrol_new cod_ep_switch =
	SOC_DAPM_SINGLE("Switch", COD_CHOP_DA, COD_CHOP_DA_EP_SHIFT, 1, 0);

static const struct snd_kcontrol_new cod_hp_switch =
	SOC_DAPM_SINGLE("Switch", COD_CHOP_DA, COD_CHOP_DA_HP_SHIFT, 1, 0);

static const struct snd_soc_dapm_widget cod_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Power", SND_SOC_NOPM, 0, 0, cod_power_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* Playback */
	SND_SOC_DAPM_AIF_IN("AIF RX", "AIF Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0, cod_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUT_DRV_E("EPDRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cod_ep_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SWITCH("Earpiece", SND_SOC_NOPM, 0, 0, &cod_ep_switch),
	SND_SOC_DAPM_OUTPUT("EPOUT"),

	SND_SOC_DAPM_OUT_DRV_E("HPDRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cod_hp_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH("Headphone", SND_SOC_NOPM, 0, 0, &cod_hp_switch),
	SND_SOC_DAPM_OUTPUT("HPOUTL"),
	SND_SOC_DAPM_OUTPUT("HPOUTR"),

	/* Capture */
	SND_SOC_DAPM_SUPPLY("VMID", SND_SOC_NOPM, 0, 0, cod_vmid_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MICBIAS LDO", COD_PD_REF, COD_PD_REF_MCB_LDO_SHIFT,
			    0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MICBIAS1", COD_PD_REF, COD_PD_REF_MCB1_SHIFT, 0,
			    NULL, 0),
	SND_SOC_DAPM_SUPPLY("MICBIAS2", COD_PD_REF, COD_PD_REF_MCB2_SHIFT, 0,
			    cod_micbias2_event,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_INPUT("IN1"),
	SND_SOC_DAPM_INPUT("IN2"),
	SND_SOC_DAPM_INPUT("IN3"),

	COD_MIC_PGA("MIC1 PGA", 1),
	COD_MIC_PGA("MIC2 PGA", 2),
	COD_MIC_PGA("MIC3 PGA", 3),

	SND_SOC_DAPM_MIXER("ADCL Mixer", SND_SOC_NOPM, 0, 0, cod_adcl_mix,
			   ARRAY_SIZE(cod_adcl_mix)),
	SND_SOC_DAPM_MIXER("ADCR Mixer", SND_SOC_NOPM, 0, 0, cod_adcr_mix,
			   ARRAY_SIZE(cod_adcr_mix)),

	SND_SOC_DAPM_ADC_E("ADC", NULL, SND_SOC_NOPM, 0, 0, cod_adc_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_AIF_OUT("AIF TX", "AIF Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route cod_routes[] = {
	{ "DAC", NULL, "AIF RX" },
	{ "DAC", NULL, "Power" },

	{ "EPDRV", NULL, "DAC" },
	{ "Earpiece", "Switch", "EPDRV" },
	{ "EPOUT", NULL, "Earpiece" },

	{ "HPDRV", NULL, "DAC" },
	{ "Headphone", "Switch", "HPDRV" },
	{ "HPOUTL", NULL, "Headphone" },
	{ "HPOUTR", NULL, "Headphone" },

	{ "MICBIAS1", NULL, "MICBIAS LDO" },
	{ "MICBIAS2", NULL, "MICBIAS LDO" },

	{ "MIC1 PGA", NULL, "IN1" },
	{ "MIC2 PGA", NULL, "IN2" },
	{ "MIC3 PGA", NULL, "IN3" },
	{ "MIC1 PGA", NULL, "VMID" },
	{ "MIC2 PGA", NULL, "VMID" },
	{ "MIC3 PGA", NULL, "VMID" },

	{ "ADCL Mixer", "MIC1 Switch", "MIC1 PGA" },
	{ "ADCL Mixer", "MIC2 Switch", "MIC2 PGA" },
	{ "ADCL Mixer", "MIC3 Switch", "MIC3 PGA" },
	{ "ADCR Mixer", "MIC1 Switch", "MIC1 PGA" },
	{ "ADCR Mixer", "MIC2 Switch", "MIC2 PGA" },
	{ "ADCR Mixer", "MIC3 Switch", "MIC3 PGA" },

	{ "ADC", NULL, "ADCL Mixer" },
	{ "ADC", NULL, "ADCR Mixer" },
	{ "ADC", NULL, "Power" },
	{ "AIF TX", NULL, "ADC" },
};

static int cod_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	unsigned int val = 0;

	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		val |= COD_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val |= COD_FORMAT_LRJ;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		val |= COD_FORMAT_BCLK_POL | COD_FORMAT_LRCLK_POL;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val |= COD_FORMAT_BCLK_POL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		val |= COD_FORMAT_LRCLK_POL;
		break;
	default:
		return -EINVAL;
	}

	return snd_soc_component_update_bits(component, COD_FORMAT,
					     COD_FORMAT_I2S | COD_FORMAT_LRJ |
					     COD_FORMAT_BCLK_POL |
					     COD_FORMAT_LRCLK_POL, val);
}

static int cod_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	int ret;

	if (params_rate(params) != 48000)
		return -EINVAL;

	/* The mixer always sends 24-bit samples */
	ret = snd_soc_component_update_bits(component, COD_FORMAT,
					    COD_FORMAT_WORD_LEN,
					    FIELD_PREP(COD_FORMAT_WORD_LEN,
						       COD_FORMAT_WORD_LEN_24));

	return ret < 0 ? ret : 0;
}

static const struct snd_soc_dai_ops cod_dai_ops = {
	.set_fmt = cod_set_fmt,
	.hw_params = cod_hw_params,
};

#define COD_FORMATS	(SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver cod_dai = {
	.name = "cod3026x-aif",
	.playback = {
		.stream_name = "AIF Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = COD_FORMATS,
	},
	.capture = {
		.stream_name = "AIF Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = COD_FORMATS,
	},
	.ops = &cod_dai_ops,
	.symmetric_rate = 1,
};

/* Average of the samples without the lowest and the highest one */
static int cod_jack_read_uv(struct cod3026x *cod, int *uv)
{
	int i, val, min = INT_MAX, max = INT_MIN, sum = 0;
	int ret;

	for (i = 0; i < COD_ADC_SAMPLES; i++) {
		ret = iio_read_channel_processed_scale(cod->jack_adc, &val,
						       1000);
		if (ret < 0)
			return ret;
		sum += val;
		min = min(min, val);
		max = max(max, val);
	}

	*uv = (sum - min - max) / (COD_ADC_SAMPLES - 2);

	return 0;
}

static void cod_jack_work(struct work_struct *work)
{
	struct cod3026x *cod = container_of(work, struct cod3026x,
					    jack_work.work);
	unsigned int status;
	int uv, type = 0;

	mutex_lock(&cod->jack_lock);

	if (regmap_read(cod->det_regmap, COD_STATUS1, &status))
		goto out;

	cod->jack_in = status & COD_STATUS1_JACK;

	if (cod->jack_in) {
		if (cod_jack_read_uv(cod, &uv))
			goto out;
		type = uv > cod->mic_uv ? SND_JACK_HEADSET : SND_JACK_HEADPHONE;
		dev_dbg(cod->dev, "jack in, %d uV\n", uv);
	}

	/* Detection block settings for each state, from the vendor driver */
	switch (type) {
	case SND_JACK_HEADSET:
		regmap_write(cod->det_regmap, COD_MIC_DET, 0x02);
		regmap_write(cod->det_regmap, COD_DET_PDB, 0x00);
		break;
	case SND_JACK_HEADPHONE:
		regmap_write(cod->det_regmap, COD_MIC_DET, 0x02);
		regmap_write(cod->det_regmap, COD_DET_PDB, 0x01);
		break;
	default:
		regmap_write(cod->det_regmap, COD_MIC_DET, 0xff);
		regmap_write(cod->det_regmap, COD_DET_PDB, 0x00);
		break;
	}

	cod->jack_type = type;
	cod->button = -1;
	snd_soc_jack_report(&cod->jack, type,
			    SND_JACK_HEADSET | SND_JACK_BTN_0 | SND_JACK_BTN_1 |
			    SND_JACK_BTN_2 | SND_JACK_BTN_3);
out:
	mutex_unlock(&cod->jack_lock);
}

static const int cod_button_bits[COD_JACK_BUTTONS] = {
	SND_JACK_BTN_0, SND_JACK_BTN_1, SND_JACK_BTN_2, SND_JACK_BTN_3,
};

static void cod_button_work(struct work_struct *work)
{
	struct cod3026x *cod = container_of(work, struct cod3026x,
					    button_work.work);
	int uv, i, button = -1;

	mutex_lock(&cod->jack_lock);

	if (cod->jack_type != SND_JACK_HEADSET)
		goto out;

	if (cod_jack_read_uv(cod, &uv))
		goto out;

	if (uv <= cod->release_uv) {
		for (i = 0; i < COD_JACK_BUTTONS; i++) {
			if (uv <= cod->button_uv[i]) {
				button = i;
				break;
			}
		}
	}

	dev_dbg(cod->dev, "button %d, %d uV\n", button, uv);

	if (button == cod->button)
		goto out;

	cod->button = button;
	snd_soc_jack_report(&cod->jack,
			    button < 0 ? 0 : cod_button_bits[button],
			    SND_JACK_BTN_0 | SND_JACK_BTN_1 |
			    SND_JACK_BTN_2 | SND_JACK_BTN_3);
out:
	mutex_unlock(&cod->jack_lock);
}

static irqreturn_t cod_irq(int irq, void *data)
{
	struct cod3026x *cod = data;
	unsigned int pend[3];
	int i;

	/* Reading the pending registers clears them */
	for (i = 0; i < ARRAY_SIZE(pend); i++)
		if (regmap_read(cod->det_regmap, COD_IRQ1PEND + i, &pend[i]))
			return IRQ_NONE;

	if ((pend[1] | pend[2]) & COD_IRQ_JACK_DET) {
		cancel_delayed_work(&cod->button_work);
		/* Let the plug settle before measuring the microphone */
		mod_delayed_work(system_dfl_wq, &cod->jack_work,
				 msecs_to_jiffies(COD_JACK_DETECT_DELAY_MS));
	} else if (pend[0] | pend[1] | pend[2]) {
		mod_delayed_work(system_dfl_wq, &cod->button_work,
				 msecs_to_jiffies(COD_BUTTON_DELAY_MS));
	} else {
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int cod_component_probe(struct snd_soc_component *component)
{
	struct cod3026x *cod = snd_soc_component_get_drvdata(component);
	int ret;

	cod->component = component;

	if (!cod->jack_adc || cod->irq <= 0)
		return 0;

	ret = snd_soc_card_jack_new(component->card, "Headset",
				    SND_JACK_HEADSET | SND_JACK_BTN_0 |
				    SND_JACK_BTN_1 | SND_JACK_BTN_2 |
				    SND_JACK_BTN_3, &cod->jack);
	if (ret)
		return ret;

	/* Button impedances of the 3.5 mm headset specification */
	snd_jack_set_key(cod->jack.jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(cod->jack.jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(cod->jack.jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(cod->jack.jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	/* Unmask the jack and button interrupts */
	ret = regmap_write(cod->det_regmap, COD_IRQ1M, 0x80);
	if (!ret)
		ret = regmap_write(cod->det_regmap, COD_IRQ2M, 0xc0);
	if (!ret)
		ret = regmap_write(cod->det_regmap, COD_IRQ3M, 0xc0);
	if (ret)
		return ret;

	ret = request_threaded_irq(cod->irq, NULL, cod_irq, IRQF_ONESHOT,
				   dev_name(cod->dev), cod);
	if (ret)
		return ret;

	/* Report a jack that was plugged in before */
	mod_delayed_work(system_dfl_wq, &cod->jack_work, 0);

	return 0;
}

static void cod_component_remove(struct snd_soc_component *component)
{
	struct cod3026x *cod = snd_soc_component_get_drvdata(component);

	if (!cod->jack_adc || cod->irq <= 0)
		return;

	free_irq(cod->irq, cod);
	cancel_delayed_work_sync(&cod->jack_work);
	cancel_delayed_work_sync(&cod->button_work);
	regmap_write(cod->det_regmap, COD_IRQ1M, 0xff);
	regmap_write(cod->det_regmap, COD_IRQ2M, 0xff);
	regmap_write(cod->det_regmap, COD_IRQ3M, 0xff);
}

static const struct snd_soc_component_driver cod_component = {
	.probe = cod_component_probe,
	.remove = cod_component_remove,
	.controls = cod_controls,
	.num_controls = ARRAY_SIZE(cod_controls),
	.dapm_widgets = cod_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cod_widgets),
	.dapm_routes = cod_routes,
	.num_dapm_routes = ARRAY_SIZE(cod_routes),
	.use_pmdown_time = 1,
	.endianness = 1,
};

static int cod_save_trims(struct cod3026x *cod)
{
	unsigned int val;
	bool nonzero = false;
	int i, ret;

	for (i = 0; i < COD_NUM_TRIMS; i++) {
		ret = regmap_read(cod->regmap, COD_TRIM_FIRST + i, &val);
		if (ret)
			return ret;
		cod->trims[i] = val;
		nonzero |= val;
	}

	/* All zero means the chip did not load them: don't restore that */
	if (!nonzero) {
		dev_warn(cod->dev, "trim registers read as zero\n");
		return 0;
	}

	cod->trims_saved = true;
	dev_dbg(cod->dev, "trims %*ph\n", COD_NUM_TRIMS, cod->trims);

	return 0;
}

static int cod_restore_trims(struct cod3026x *cod)
{
	int i, ret;

	if (!cod->trims_saved)
		return 0;

	for (i = 0; i < COD_NUM_TRIMS; i++) {
		ret = regmap_write(cod->regmap, COD_TRIM_FIRST + i,
				   cod->trims[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int cod_runtime_suspend(struct device *dev)
{
	struct cod3026x *cod = dev_get_drvdata(dev);

	regcache_cache_only(cod->regmap, true);
	regcache_mark_dirty(cod->regmap);
	clk_disable_unprepare(cod->mclk);
	regulator_disable(cod->vdd);

	return 0;
}

static int cod_runtime_resume(struct device *dev)
{
	struct cod3026x *cod = dev_get_drvdata(dev);
	int ret;

	ret = regulator_enable(cod->vdd);
	if (ret)
		return ret;

	ret = clk_prepare_enable(cod->mclk);
	if (ret)
		goto err_vdd;

	regcache_cache_only(cod->regmap, false);

	ret = regmap_set_bits(cod->regmap, COD_DET_ON, COD_DET_ON_PDB_JD_CLK);
	if (ret)
		goto err_clk;

	/* The chip loads its trims and resets the register file */
	usleep_range(15000, 16000);

	/* Digital power first, as the vendor driver does */
	ret = regcache_sync_region(cod->regmap, COD_DIGITAL_POWER,
				   COD_DIGITAL_POWER);
	if (!ret)
		ret = regcache_sync(cod->regmap);
	if (ret)
		goto err_clk;

	if (cod->trims_saved)
		ret = cod_restore_trims(cod);
	else
		ret = cod_save_trims(cod);
	if (ret)
		goto err_clk;

	return 0;

err_clk:
	regcache_cache_only(cod->regmap, true);
	clk_disable_unprepare(cod->mclk);
err_vdd:
	regulator_disable(cod->vdd);
	return ret;
}

static int cod_fill_cache(struct cod3026x *cod)
{
	unsigned int reg, val;
	int ret;

	for (reg = 0; reg <= COD_TRIM_LAST; reg++) {
		if (!cod_readable_reg(cod->dev, reg) ||
		    cod_volatile_reg(cod->dev, reg))
			continue;
		ret = regmap_read(cod->regmap, reg, &val);
		if (ret)
			return ret;
	}

	return 0;
}

/* Settings from the vendor driver, applied when no tuning file is loaded */
static int cod_init_chip(struct cod3026x *cod)
{
	const struct reg_sequence detect_seq[] = {
		{ COD_DET_ON, 0x42, 1000 },
		{ COD_DET_ON, 0x43 },
		{ COD_MIC_DET, 0xff },
		{ COD_DET_TIME, 0xff },
		{ COD_LDO_DIG, 0x0f },
		{ COD_KEY_TIME, 0xf2 },
		{ COD_MICBIAS2, 0x0b },
		/* jack pop resistor 2 Mohm, jack threshold 150 kohm */
		{ COD_JACK_DET2, 0x0d },
		{ COD_MIC_BIAS,
		  FIELD_PREP(COD_MIC_BIAS_MCB2, cod->mcb2) |
		  FIELD_PREP(COD_MIC_BIAS_LDO, cod->mcb_ldo) },
	};
	const struct reg_sequence core_seq[] = {
		/* ADC high-pass filter at 100 Hz */
		{ COD_ADC1, 0x0c },
		/* dynamic noise control start gain */
		{ COD_DNC7, 0x1a },
		/* charge pump frequency */
		{ COD_CTRL_CP, 0x02 },
		/* MIC3 trims */
		{ COD_AD_MIC3N_TRIM1, 0x11 },
		{ COD_MIC_ON, 0x01 },
		/* all outputs off */
		{ COD_CHOP_DA, 0x00 },
		/* charge pump from VDD, as the stock media paths */
		{ COD_CTRL_EP, 0x00 },
		/* zero crossing detection on, shortest timeout */
		{ COD_DNC9, 0x81 },
		/* 0 dB digital volume instead of the +12 dB reset value */
		{ COD_DAC_L_VOL, 0x18 },
		{ COD_DAC_R_VOL, 0x18 },
		{ COD_VOL_HPL, 0x1a },
		{ COD_VOL_HPR, 0x1a },
	};
	int ret;

	ret = regmap_multi_reg_write(cod->regmap, detect_seq,
				     ARRAY_SIZE(detect_seq));
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(cod->regmap, core_seq,
				     ARRAY_SIZE(core_seq));
	if (ret)
		return ret;

	ret = regmap_update_bits(cod->regmap, COD_DNC2, COD_DNC2_MAX_GAIN, 12);
	if (ret)
		return ret;

	ret = regmap_update_bits(cod->regmap, COD_CTRL_REF, COD_CTRL_REF_MCB1,
				 FIELD_PREP(COD_CTRL_REF_MCB1, cod->mcb1));
	if (ret)
		return ret;

	/* ADC chopping clock at 1/4 */
	return regmap_update_bits(cod->regmap, COD_CLK1_AD, COD_CLK1_AD_CHCLK,
				  0);
}

static int cod_parse_bias(struct device *dev, const char *prop,
			  const u32 *table, int count, u32 def, u8 *code)
{
	u32 uv = def;
	int i;

	device_property_read_u32(dev, prop, &uv);

	for (i = 0; i < count; i++) {
		if (table[i] == uv) {
			*code = i;
			return 0;
		}
	}

	return dev_err_probe(dev, -EINVAL, "invalid %s: %u\n", prop, uv);
}

/* Register codes; code 0 of the bias voltages is not documented */
static const u32 cod_mcb_uv[] = { 0, 2800000, 2600000, 3000000 };
static const u32 cod_mcb_ldo_uv[] = { 2500000, 2800000, 3000000, 3300000 };

#define COD_PROP_MIC		"samsung,jack-mic-threshold-microvolt"
#define COD_PROP_RELEASE	"samsung,jack-button-release-microvolt"
#define COD_PROP_BUTTONS	"samsung,jack-button-thresholds-microvolt"

/* Jack detection needs the interrupt and the ADC channel of the jack */
static int cod_parse_jack(struct cod3026x *cod, int irq)
{
	struct device *dev = cod->dev;
	int ret;

	cod->irq = irq;
	if (irq <= 0)
		return 0;

	cod->jack_adc = devm_iio_channel_get(dev, "jack");
	if (IS_ERR(cod->jack_adc)) {
		ret = PTR_ERR(cod->jack_adc);
		cod->jack_adc = NULL;
		if (ret == -ENODEV)
			return 0;
		return dev_err_probe(dev, ret, "Failed to get jack ADC\n");
	}

	ret = device_property_read_u32(dev, COD_PROP_MIC, &cod->mic_uv);
	if (!ret)
		ret = device_property_read_u32(dev, COD_PROP_RELEASE,
					       &cod->release_uv);
	if (!ret)
		ret = device_property_read_u32_array(dev, COD_PROP_BUTTONS,
						     cod->button_uv,
						     COD_JACK_BUTTONS);
	if (ret)
		return dev_err_probe(dev, ret, "Missing jack thresholds\n");

	return 0;
}

static int cod_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct cod3026x *cod;
	int ret;

	cod = devm_kzalloc(dev, sizeof(*cod), GFP_KERNEL);
	if (!cod)
		return -ENOMEM;

	cod->dev = dev;
	i2c_set_clientdata(i2c, cod);
	INIT_DELAYED_WORK(&cod->adc_unmute_work, cod_adc_unmute_work);
	INIT_DELAYED_WORK(&cod->jack_work, cod_jack_work);
	INIT_DELAYED_WORK(&cod->button_work, cod_button_work);
	mutex_init(&cod->jack_lock);

	ret = cod_parse_bias(dev, "samsung,micbias1-microvolt", cod_mcb_uv,
			     ARRAY_SIZE(cod_mcb_uv), 3000000, &cod->mcb1);
	if (ret)
		return ret;
	ret = cod_parse_bias(dev, "samsung,micbias2-microvolt", cod_mcb_uv,
			     ARRAY_SIZE(cod_mcb_uv), 3000000, &cod->mcb2);
	if (ret)
		return ret;
	ret = cod_parse_bias(dev, "samsung,micbias-ldo-microvolt",
			     cod_mcb_ldo_uv, ARRAY_SIZE(cod_mcb_ldo_uv),
			     3300000, &cod->mcb_ldo);
	if (ret)
		return ret;

	cod->det_regmap = devm_regmap_init_i2c(i2c, &cod_det_regmap_config);
	if (IS_ERR(cod->det_regmap))
		return dev_err_probe(dev, PTR_ERR(cod->det_regmap),
				     "Failed to init detection regmap\n");

	ret = cod_parse_jack(cod, i2c->irq);
	if (ret)
		return ret;

	cod->regmap = devm_regmap_init_i2c(i2c, &cod_regmap_config);
	if (IS_ERR(cod->regmap))
		return dev_err_probe(dev, PTR_ERR(cod->regmap),
				     "Failed to init regmap\n");

	cod->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(cod->vdd))
		return dev_err_probe(dev, PTR_ERR(cod->vdd),
				     "Failed to get vdd\n");

	cod->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(cod->mclk))
		return dev_err_probe(dev, PTR_ERR(cod->mclk),
				     "Failed to get mclk\n");

	regcache_cache_only(cod->regmap, true);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to power up\n");

	ret = cod_fill_cache(cod);
	if (!ret)
		ret = cod_init_chip(cod);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to initialise the chip\n");
		goto err_put;
	}

	ret = devm_snd_soc_register_component(dev, &cod_component, &cod_dai,
					      1);
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

static void cod_remove(struct i2c_client *i2c)
{
	struct cod3026x *cod = i2c_get_clientdata(i2c);

	cancel_delayed_work_sync(&cod->adc_unmute_work);
}

static const struct dev_pm_ops cod_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(cod_runtime_suspend, cod_runtime_resume, NULL)
};

static const struct of_device_id cod_of_match[] = {
	{ .compatible = "samsung,cod3026x" },
	{ }
};
MODULE_DEVICE_TABLE(of, cod_of_match);

static struct i2c_driver cod_driver = {
	.driver = {
		.name = "cod3026x",
		.of_match_table = cod_of_match,
		.pm = pm_ptr(&cod_pm_ops),
	},
	.probe = cod_probe,
	.remove = cod_remove,
};
module_i2c_driver(cod_driver);

MODULE_DESCRIPTION("Samsung COD3026X audio codec driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
