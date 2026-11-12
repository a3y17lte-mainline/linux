// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S2MU004 USB Type-C port controller and USB PD PHY
 *
 * The CC state machine (including DRP toggling), BMC coding, CRC, GoodCRC
 * and the MessageID counter are handled by the chip; TCPM runs the policy.
 * VBUS is sensed and sourced by the S2MU004 charger: its power supply (the
 * one listing this device in "power-supplies") reports VBUS, and its OTG
 * boost regulator ("vbus-supply") sources it.
 *
 * Register layout and sequences follow the Samsung vendor driver
 * (drivers/ccic/s2mu004-usbpd.c).
 *
 * Copyright (c) 2026 Martin Štrobl <recordyletsplays@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/workqueue.h>

#define S2MU004_REG_PD_CTRL		0x01
#define S2MU004_PD_LPM_EN		BIT(1)
#define S2MU004_REG_PD_CTRL2		0x02
#define S2MU004_PD_CC_OCP		GENMASK(7, 4)
#define S2MU004_PD_CC_OCP_575MV		15
#define S2MU004_REG_PLUG_CTRL_PORT	0x18
#define S2MU004_PORT_MODE		GENMASK(1, 0)
#define S2MU004_PORT_MODE_DFP		1
#define S2MU004_PORT_MODE_UFP		2
#define S2MU004_PORT_MODE_DRP		3
#define S2MU004_PORT_RP_SEL		GENMASK(5, 4)
#define S2MU004_PORT_RP_OFF		0
#define S2MU004_PORT_RP_80UA		1
#define S2MU004_PORT_RP_180UA		2
#define S2MU004_REG_PLUG_CTRL_MSG	0x19
#define S2MU004_MSG_DATA_ROLE_DFP	BIT(5)
#define S2MU004_MSG_POWER_ROLE_SRC	BIT(6)
#define S2MU004_REG_SET_RD		0x1e
#define S2MU004_REG_SET_RP		0x1f
#define S2MU004_REG_PLUG_CTRL_RPRD	0x27
#define S2MU004_RPRD_FSM_MANUAL_EN	BIT(2)
#define S2MU004_REG_PLUG_CTRL_CC12	0x28
#define S2MU004_CC12_FSM_INPUT		GENMASK(3, 0)
#define S2MU004_CC12_FSM_ATTACHED_SNK	2
#define S2MU004_CC12_CC_MANUAL_EN	BIT(4)
#define S2MU004_CC12_CC1_ON		BIT(5)
#define S2MU004_CC12_CC2_ON		BIT(6)
#define S2MU004_CC12_CC_MASK		GENMASK(6, 4)
#define S2MU004_REG_INT_MASK0		0x3e
#define S2MU004_REG_MSG_SEND_CON	0x90
#define S2MU004_SEND_CON_SEND		BIT(0)
#define S2MU004_SEND_CON_OP_MODE	BIT(1)
#define S2MU004_SEND_CON_SOP		GENMASK(4, 2)
#define S2MU004_SEND_CON_SOP_HARD_RESET	5
#define S2MU004_REG_MSG_HEADER		0x91	/* 2 bytes + 7 data objects */
#define S2MU004_REG_PLUG_MON1		0xb3
#define S2MU004_MON1_CC1		GENMASK(2, 0)
#define S2MU004_MON1_CC2		GENMASK(5, 3)
#define S2MU004_REG_PLUG_MON2		0xb4
#define S2MU004_MON2_STATE		GENMASK(3, 1)
#define S2MU004_MON2_SINK		0x6	/* attach done + sink */
#define S2MU004_MON2_SOURCE		0xa	/* attach done + source */
#define S2MU004_REG_INT_STATUS0		0xe0	/* 7 registers, clear on read */
#define S2MU004_REG_ETC			0xf7
#define S2MU004_ETC_SOFT_RESET		BIT(1)
#define S2MU004_REG_ID_MONITOR		0xf8
#define S2MU004_ID_MONITOR_MSG_ID	GENMASK(2, 0)

#define S2MU004_NUM_INT			7

/* Interrupt status bits (STATUS4, STATUS5) */
#define S2MU004_INT4_MSG_ERROR		BIT(1)
#define S2MU004_INT4_MSG_SENT		BIT(2)
#define S2MU004_INT4_MSG_PASS		BIT(3)
#define S2MU004_INT4_USB_DETACH		BIT(4)
#define S2MU004_INT4_PLUG		BIT(5)
#define S2MU004_INT4_CC12_DET		BIT(6)
#define S2MU004_INT5_HARD_RESET		BIT(2)

/* CC comparator thresholds (PLUG_CTRL_SET_RD/SET_RP codes) */
#define S2MU004_THRESHOLD_214MV		4
#define S2MU004_THRESHOLD_428MV		9
#define S2MU004_THRESHOLD_685MV		20
#define S2MU004_THRESHOLD_1328MV	35
#define S2MU004_THRESHOLD_1628MV	42
#define S2MU004_THRESHOLD_2057MV	52

/* PLUG_MON1 per-CC codes */
#define S2MU004_MON_CC_RA		1
#define S2MU004_MON_CC_CONNECTED	2	/* Rd as source, Rp as sink */

/* The chip enters low-power mode after a received hard reset */
#define S2MU004_LPM_RECOVER_MS		300

enum s2mu004_attach {
	S2MU004_DETACHED,
	S2MU004_ATTACHED_SNK,
	S2MU004_ATTACHED_SRC,
};

struct s2mu004_tcpc {
	struct device *dev;
	struct regmap *regmap;
	struct tcpc_dev tcpc_dev;
	struct tcpm_port *tcpm_port;
	struct mutex lock;		/* chip registers and the state below */

	/* hardware attach state, from the last PLUG/DETACH interrupt */
	enum s2mu004_attach attached;
	enum typec_cc_status cc1, cc2;
	u8 int_mask[S2MU004_NUM_INT];
	bool pd_rx_on;
	bool tx_pending;
	struct delayed_work lpm_work;

	/* VBUS: sourced through the regulator, sensed through the charger */
	struct regulator *vbus;
	bool vbus_on;
	struct notifier_block psy_nb;
	struct work_struct vbus_work;
	struct power_supply *charger;
	spinlock_t charger_lock;	/* charger */
	bool vbus_present;
};

static bool s2mu004_tcpc_precious_reg(struct device *dev, unsigned int reg)
{
	return reg >= S2MU004_REG_INT_STATUS0 &&
	       reg < S2MU004_REG_INT_STATUS0 + S2MU004_NUM_INT;
}

static const struct regmap_config s2mu004_tcpc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = S2MU004_REG_ID_MONITOR,
	.precious_reg = s2mu004_tcpc_precious_reg,
	.cache_type = REGCACHE_NONE,
};

static struct s2mu004_tcpc *tcpc_to_chip(struct tcpc_dev *dev)
{
	return container_of(dev, struct s2mu004_tcpc, tcpc_dev);
}

static int s2mu004_write_masks(struct s2mu004_tcpc *chip)
{
	return regmap_bulk_write(chip->regmap, S2MU004_REG_INT_MASK0,
				 chip->int_mask, S2MU004_NUM_INT);
}

/* Unmask the attach/detach and hard reset interrupts, plus messages with pd_rx */
static int s2mu004_update_masks(struct s2mu004_tcpc *chip)
{
	memset(chip->int_mask, 0xff, sizeof(chip->int_mask));
	chip->int_mask[4] &= ~(S2MU004_INT4_PLUG | S2MU004_INT4_USB_DETACH);
	chip->int_mask[5] &= ~S2MU004_INT5_HARD_RESET;

	/*
	 * The chip also sorts received messages into STATUS0-3 bits, each
	 * followed by MSG_PASS: MSG_PASS alone covers every message.
	 */
	/*
	 * CC12_DET fires on every DRP toggle; it is only needed to catch the
	 * chip entering low-power mode after a received hard reset.
	 */
	if (chip->pd_rx_on)
		chip->int_mask[4] &= ~(S2MU004_INT4_MSG_ERROR | S2MU004_INT4_MSG_SENT |
				       S2MU004_INT4_MSG_PASS | S2MU004_INT4_CC12_DET);

	return s2mu004_write_masks(chip);
}

static int s2mu004_set_thresholds(struct s2mu004_tcpc *chip, unsigned int rd,
				  unsigned int rp)
{
	int ret;

	ret = regmap_write(chip->regmap, S2MU004_REG_SET_RD, rd);
	if (ret)
		return ret;

	return regmap_write(chip->regmap, S2MU004_REG_SET_RP, rp);
}

static int s2mu004_set_port(struct s2mu004_tcpc *chip, unsigned int mode,
			    unsigned int rp)
{
	int ret;

	/* the comparator thresholds go with the Rp current */
	if (rp == S2MU004_PORT_RP_80UA)
		ret = s2mu004_set_thresholds(chip, S2MU004_THRESHOLD_214MV,
					     S2MU004_THRESHOLD_1628MV);
	else
		ret = s2mu004_set_thresholds(chip, S2MU004_THRESHOLD_428MV,
					     S2MU004_THRESHOLD_2057MV);
	if (ret)
		return ret;

	return regmap_update_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_PORT,
				  S2MU004_PORT_MODE | S2MU004_PORT_RP_SEL,
				  FIELD_PREP(S2MU004_PORT_MODE, mode) |
				  FIELD_PREP(S2MU004_PORT_RP_SEL, rp));
}

static int s2mu004_soft_reset(struct s2mu004_tcpc *chip)
{
	int ret;

	ret = regmap_write(chip->regmap, S2MU004_REG_ETC, S2MU004_ETC_SOFT_RESET);
	if (ret)
		return ret;

	return regmap_write(chip->regmap, S2MU004_REG_ETC, 0);
}

/* Normal (not low-power) operation with automatic CC handling */
static int s2mu004_chip_setup(struct s2mu004_tcpc *chip)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, S2MU004_REG_PD_CTRL2, S2MU004_PD_CC_OCP,
				 FIELD_PREP(S2MU004_PD_CC_OCP, S2MU004_PD_CC_OCP_575MV));
	if (ret)
		return ret;

	ret = s2mu004_set_thresholds(chip, S2MU004_THRESHOLD_428MV,
				     S2MU004_THRESHOLD_2057MV);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, S2MU004_REG_PLUG_CTRL_RPRD, 0);
	if (ret)
		return ret;

	/* no BMC until TCPM enables PD reception: no GoodCRC it cannot follow */
	ret = regmap_write(chip->regmap, S2MU004_REG_PLUG_CTRL_CC12,
			   chip->pd_rx_on ? 0 : S2MU004_CC12_CC_MANUAL_EN);
	if (ret)
		return ret;

	ret = regmap_clear_bits(chip->regmap, S2MU004_REG_PD_CTRL, S2MU004_PD_LPM_EN);
	if (ret)
		return ret;

	return s2mu004_update_masks(chip);
}

static void s2mu004_cc_from_mon1(struct s2mu004_tcpc *chip, unsigned int mon1,
				 enum typec_cc_status connected)
{
	unsigned int code[2] = {
		FIELD_GET(S2MU004_MON1_CC1, mon1),
		FIELD_GET(S2MU004_MON1_CC2, mon1),
	};
	enum typec_cc_status cc[2];
	int i;

	for (i = 0; i < 2; i++) {
		if (code[i] == S2MU004_MON_CC_CONNECTED)
			cc[i] = connected;
		else if (code[i] == S2MU004_MON_CC_RA &&
			 chip->attached == S2MU004_ATTACHED_SRC)
			cc[i] = TYPEC_CC_RA;
		else
			cc[i] = TYPEC_CC_OPEN;
	}

	chip->cc1 = cc[0];
	chip->cc2 = cc[1];
}

/*
 * PLUG_MON1 reads S2MU004_MON_CC_CONNECTED for a CC whose voltage lies
 * between the SET_RD and SET_RP thresholds. As a sink, find the source's
 * Rp current from the voltage on the connected CC (vRd-USB up to 0.61 V,
 * vRd-1.5 up to 1.16 V, vRd-3.0 above) by lowering SET_RP, with the state
 * machine held in the attached-sink state.
 */
static int s2mu004_cc_below(struct s2mu004_tcpc *chip, u32 cc_mask,
			    unsigned int threshold, bool *below)
{
	unsigned int mon1;
	int ret;

	ret = regmap_write(chip->regmap, S2MU004_REG_SET_RP, threshold);
	if (ret)
		return ret;

	msleep(20);
	ret = regmap_read(chip->regmap, S2MU004_REG_PLUG_MON1, &mon1);
	if (ret)
		return ret;

	dev_dbg(chip->dev, "Rp check: MON1 %#04x below code %u\n", mon1, threshold);
	*below = field_get(cc_mask, mon1) == S2MU004_MON_CC_CONNECTED;

	return 0;
}

static enum typec_cc_status s2mu004_rp_level(struct s2mu004_tcpc *chip,
					     unsigned int mon1)
{
	u32 cc_mask = FIELD_GET(S2MU004_MON1_CC1, mon1) == S2MU004_MON_CC_CONNECTED ?
		      S2MU004_MON1_CC1 : S2MU004_MON1_CC2;
	enum typec_cc_status level = TYPEC_CC_RP_DEF;
	bool below;
	int ret;

	ret = regmap_update_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_CC12,
				 S2MU004_CC12_FSM_INPUT, S2MU004_CC12_FSM_ATTACHED_SNK);
	if (!ret)
		ret = regmap_set_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_RPRD,
				      S2MU004_RPRD_FSM_MANUAL_EN);
	if (!ret)
		ret = regmap_write(chip->regmap, S2MU004_REG_SET_RD,
				   S2MU004_THRESHOLD_214MV);
	if (ret)
		goto restore;

	ret = s2mu004_cc_below(chip, cc_mask, S2MU004_THRESHOLD_685MV, &below);
	if (ret || below)
		goto restore;

	ret = s2mu004_cc_below(chip, cc_mask, S2MU004_THRESHOLD_1328MV, &below);
	if (!ret)
		level = below ? TYPEC_CC_RP_1_5 : TYPEC_CC_RP_3_0;

restore:
	s2mu004_set_thresholds(chip, S2MU004_THRESHOLD_428MV, S2MU004_THRESHOLD_2057MV);
	msleep(20);
	regmap_clear_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_RPRD,
			  S2MU004_RPRD_FSM_MANUAL_EN);
	regmap_clear_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_CC12,
			  S2MU004_CC12_FSM_INPUT);

	if (ret) {
		dev_err(chip->dev, "Rp level check failed: %d\n", ret);
		return TYPEC_CC_RP_DEF;
	}

	dev_dbg(chip->dev, "Rp level %d\n", level);

	return level;
}

static void s2mu004_detached(struct s2mu004_tcpc *chip)
{
	chip->attached = S2MU004_DETACHED;
	chip->cc1 = TYPEC_CC_OPEN;
	chip->cc2 = TYPEC_CC_OPEN;
	chip->tx_pending = false;
	/* as the vendor driver does, for the next hardware detection */
	regmap_write(chip->regmap, S2MU004_REG_MSG_SEND_CON, 0);
}

static void s2mu004_attached(struct s2mu004_tcpc *chip)
{
	unsigned int mon1, mon2;

	if (regmap_read(chip->regmap, S2MU004_REG_PLUG_MON2, &mon2) ||
	    regmap_read(chip->regmap, S2MU004_REG_PLUG_MON1, &mon1))
		return;

	dev_dbg(chip->dev, "attach: MON1 %#04x MON2 %#04x\n", mon1, mon2);

	switch (mon2 & S2MU004_MON2_STATE) {
	case S2MU004_MON2_SINK:
		chip->attached = S2MU004_ATTACHED_SNK;
		s2mu004_cc_from_mon1(chip, mon1, s2mu004_rp_level(chip, mon1));

		/* the check above can lose a short attach */
		if (regmap_read(chip->regmap, S2MU004_REG_PLUG_MON2, &mon2) ||
		    (mon2 & S2MU004_MON2_STATE) != S2MU004_MON2_SINK)
			s2mu004_detached(chip);
		break;
	case S2MU004_MON2_SOURCE:
		chip->attached = S2MU004_ATTACHED_SRC;
		s2mu004_cc_from_mon1(chip, mon1, TYPEC_CC_RD);
		break;
	default:
		s2mu004_detached(chip);
		break;
	}
}

static void s2mu004_receive(struct s2mu004_tcpc *chip)
{
	struct pd_message msg = {};
	unsigned int cnt;
	u16 header;
	int ret;

	ret = regmap_bulk_read(chip->regmap, S2MU004_REG_MSG_HEADER, &msg.header, 2);
	if (ret)
		goto err;

	header = le16_to_cpu(msg.header);
	cnt = pd_header_cnt(header);
	if (cnt) {
		ret = regmap_bulk_read(chip->regmap, S2MU004_REG_MSG_HEADER + 2,
				       msg.payload, cnt * 4);
		if (ret)
			goto err;
	}

	dev_dbg(chip->dev, "RX header %#06x\n", header);
	tcpm_pd_receive(chip->tcpm_port, &msg, TCPC_TX_SOP);
	return;

err:
	dev_err(chip->dev, "failed to read a PD message: %d\n", ret);
}

static irqreturn_t s2mu004_tcpc_irq(int irq, void *data)
{
	struct s2mu004_tcpc *chip = data;
	u8 st[S2MU004_NUM_INT];
	bool cc_changed = false;
	int ret;

	mutex_lock(&chip->lock);

	ret = regmap_bulk_read(chip->regmap, S2MU004_REG_INT_STATUS0, st, sizeof(st));
	if (ret) {
		dev_err(chip->dev, "failed to read interrupt status: %d\n", ret);
		mutex_unlock(&chip->lock);
		return IRQ_NONE;
	}

	dev_dbg(chip->dev, "IRQ %7ph\n", st);

	if (st[5] & S2MU004_INT5_HARD_RESET) {
		/* PD comms off and a PHY reset, as the vendor driver does */
		regmap_update_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_CC12,
				   S2MU004_CC12_CC_MASK, S2MU004_CC12_CC_MANUAL_EN);
		s2mu004_soft_reset(chip);
		s2mu004_update_masks(chip);
		chip->tx_pending = false;
		tcpm_pd_hard_reset(chip->tcpm_port);
		goto out;
	}

	/* attach and detach can arrive together: detach first */
	if (st[4] & S2MU004_INT4_USB_DETACH) {
		s2mu004_detached(chip);
		cc_changed = true;
	}

	if (st[4] & S2MU004_INT4_PLUG) {
		s2mu004_attached(chip);
		cc_changed = true;
	}

	if (st[4] & S2MU004_INT4_CC12_DET) {
		unsigned int val;

		if (!regmap_read(chip->regmap, S2MU004_REG_PD_CTRL, &val) &&
		    (val & S2MU004_PD_LPM_EN))
			mod_delayed_work(system_dfl_wq, &chip->lpm_work,
					 msecs_to_jiffies(S2MU004_LPM_RECOVER_MS));
	}

	if (cc_changed)
		tcpm_cc_change(chip->tcpm_port);

	/* the reply to a message must not overtake its GoodCRC */
	if (chip->tx_pending) {
		if (st[4] & S2MU004_INT4_MSG_SENT) {
			chip->tx_pending = false;
			tcpm_pd_transmit_complete(chip->tcpm_port, TCPC_TX_SUCCESS);
		} else if (st[4] & S2MU004_INT4_MSG_ERROR) {
			chip->tx_pending = false;
			tcpm_pd_transmit_complete(chip->tcpm_port, TCPC_TX_FAILED);
		}
	}

	if (chip->pd_rx_on && (st[4] & S2MU004_INT4_MSG_PASS))
		s2mu004_receive(chip);

out:
	mutex_unlock(&chip->lock);

	return IRQ_HANDLED;
}

static void s2mu004_lpm_work(struct work_struct *work)
{
	struct s2mu004_tcpc *chip = container_of(to_delayed_work(work),
						 struct s2mu004_tcpc, lpm_work);

	mutex_lock(&chip->lock);
	dev_dbg(chip->dev, "leaving low-power mode after a hard reset\n");
	s2mu004_chip_setup(chip);
	mutex_unlock(&chip->lock);
}

static int s2mu004_tcpm_init(struct tcpc_dev *dev)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);
	u8 st[S2MU004_NUM_INT];
	int ret;

	mutex_lock(&chip->lock);

	/* the bootloader leaves events pending: the IRQ line is low */
	ret = regmap_bulk_read(chip->regmap, S2MU004_REG_INT_STATUS0, st, sizeof(st));
	if (!ret)
		ret = s2mu004_soft_reset(chip);
	if (!ret)
		ret = s2mu004_chip_setup(chip);

	mutex_unlock(&chip->lock);

	return ret;
}

static int s2mu004_tcpm_get_vbus(struct tcpc_dev *dev)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);

	return READ_ONCE(chip->vbus_on) || READ_ONCE(chip->vbus_present);
}

static int s2mu004_tcpm_set_cc(struct tcpc_dev *dev, enum typec_cc_status cc)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);
	int ret = 0;

	mutex_lock(&chip->lock);

	switch (cc) {
	case TYPEC_CC_RD:
		/* the hardware state machine got there by itself */
		if (chip->attached == S2MU004_ATTACHED_SNK)
			break;
		ret = s2mu004_set_port(chip, S2MU004_PORT_MODE_UFP, S2MU004_PORT_RP_180UA);
		break;
	case TYPEC_CC_RP_DEF:
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		if (chip->attached == S2MU004_ATTACHED_SRC)
			break;
		/* no 3.0 A Rp: 1.5 A is the most the OTG boost supplies anyway */
		ret = s2mu004_set_port(chip, S2MU004_PORT_MODE_DFP,
				       cc == TYPEC_CC_RP_DEF ? S2MU004_PORT_RP_80UA :
							       S2MU004_PORT_RP_180UA);
		break;
	case TYPEC_CC_OPEN:
		ret = s2mu004_set_port(chip, S2MU004_PORT_MODE_DFP, S2MU004_PORT_RP_OFF);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&chip->lock);

	return ret;
}

static int s2mu004_tcpm_get_cc(struct tcpc_dev *dev, enum typec_cc_status *cc1,
			       enum typec_cc_status *cc2)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);

	mutex_lock(&chip->lock);
	*cc1 = chip->cc1;
	*cc2 = chip->cc2;
	mutex_unlock(&chip->lock);

	return 0;
}

/* BMC uses the CC the chip found attached, see s2mu004_tcpm_set_pd_rx() */
static int s2mu004_tcpm_set_polarity(struct tcpc_dev *dev,
				     enum typec_cc_polarity polarity)
{
	return 0;
}

static int s2mu004_tcpm_set_vconn(struct tcpc_dev *dev, bool on)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);

	/*
	 * The board has no VCONN supply. Report success so that a partner
	 * with Ra (a cable or an adapter) still attaches.
	 */
	if (on)
		dev_dbg(chip->dev, "VCONN requested, not supported\n");

	return 0;
}

static int s2mu004_tcpm_set_vbus(struct tcpc_dev *dev, bool on, bool charge)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);
	struct regulator *vbus;
	int ret = 0;

	mutex_lock(&chip->lock);

	if (on == chip->vbus_on)
		goto out;

	/*
	 * Taken on first use: the charger providing it waits for this
	 * device's power supply before it probes.
	 */
	if (!chip->vbus) {
		vbus = regulator_get(chip->dev, "vbus");
		if (IS_ERR(vbus)) {
			ret = PTR_ERR(vbus);
			goto out;
		}
		chip->vbus = vbus;
	}

	ret = on ? regulator_enable(chip->vbus) : regulator_disable(chip->vbus);
	if (!ret) {
		WRITE_ONCE(chip->vbus_on, on);
		/* the charger does not report its own boost as VBUS */
		tcpm_vbus_change(chip->tcpm_port);
	}

out:
	mutex_unlock(&chip->lock);

	return ret;
}

static int s2mu004_tcpm_set_pd_rx(struct tcpc_dev *dev, bool on)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);
	unsigned int cc;
	int ret;

	mutex_lock(&chip->lock);

	/*
	 * As a sink, BMC goes over the CC selected by hand (as the vendor
	 * driver does): the one the chip found attached, since a hard reset
	 * during attach can come before TCPM sets the polarity. As a source
	 * the chip selects it. Off: no CC.
	 */
	if (!on)
		cc = S2MU004_CC12_CC_MANUAL_EN;
	else if (chip->attached != S2MU004_ATTACHED_SNK)
		cc = 0;
	else if (chip->cc1 != TYPEC_CC_OPEN)
		cc = S2MU004_CC12_CC_MANUAL_EN | S2MU004_CC12_CC1_ON;
	else
		cc = S2MU004_CC12_CC_MANUAL_EN | S2MU004_CC12_CC2_ON;

	ret = regmap_update_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_CC12,
				 S2MU004_CC12_CC_MASK, cc);
	if (!ret) {
		chip->pd_rx_on = on;
		ret = s2mu004_update_masks(chip);
	}

	mutex_unlock(&chip->lock);

	return ret;
}

static int s2mu004_tcpm_set_roles(struct tcpc_dev *dev, bool attached,
				  enum typec_role pwr, enum typec_data_role data)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);
	unsigned int val = 0;
	int ret;

	if (pwr == TYPEC_SOURCE)
		val |= S2MU004_MSG_POWER_ROLE_SRC;
	if (data == TYPEC_HOST)
		val |= S2MU004_MSG_DATA_ROLE_DFP;

	mutex_lock(&chip->lock);
	ret = regmap_update_bits(chip->regmap, S2MU004_REG_PLUG_CTRL_MSG,
				 S2MU004_MSG_POWER_ROLE_SRC | S2MU004_MSG_DATA_ROLE_DFP,
				 val);
	mutex_unlock(&chip->lock);

	return ret;
}

static int s2mu004_tcpm_start_toggling(struct tcpc_dev *dev,
				       enum typec_port_type port_type,
				       enum typec_cc_status cc)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);
	unsigned int mode, mon2, rp = S2MU004_PORT_RP_180UA;
	int ret;

	switch (port_type) {
	case TYPEC_PORT_SRC:
		mode = S2MU004_PORT_MODE_DFP;
		if (cc == TYPEC_CC_RP_DEF)
			rp = S2MU004_PORT_RP_80UA;
		break;
	case TYPEC_PORT_SNK:
		mode = S2MU004_PORT_MODE_UFP;
		break;
	case TYPEC_PORT_DRP:
		mode = S2MU004_PORT_MODE_DRP;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&chip->lock);

	s2mu004_detached(chip);
	ret = regmap_write(chip->regmap, S2MU004_REG_PLUG_CTRL_RPRD, 0);
	if (!ret)
		ret = regmap_write(chip->regmap, S2MU004_REG_PLUG_CTRL_CC12,
				   S2MU004_CC12_CC_MANUAL_EN);
	if (!ret)
		ret = s2mu004_set_port(chip, mode, rp);
	if (!ret)
		ret = regmap_clear_bits(chip->regmap, S2MU004_REG_PD_CTRL,
					S2MU004_PD_LPM_EN);

	/*
	 * Changing the mode does not raise an attach interrupt when the
	 * chip is attached already, e.g. after TCPM's port reset.
	 */
	if (!ret)
		ret = regmap_read(chip->regmap, S2MU004_REG_PLUG_MON2, &mon2);
	if (!ret && (mon2 & S2MU004_MON2_STATE)) {
		s2mu004_attached(chip);
		if (chip->attached != S2MU004_DETACHED)
			tcpm_cc_change(chip->tcpm_port);
	}

	mutex_unlock(&chip->lock);

	return ret;
}

static int s2mu004_send_hard_reset(struct s2mu004_tcpc *chip)
{
	unsigned int con = S2MU004_SEND_CON_OP_MODE |
			   FIELD_PREP(S2MU004_SEND_CON_SOP, S2MU004_SEND_CON_SOP_HARD_RESET);
	int ret;

	ret = regmap_write(chip->regmap, S2MU004_REG_MSG_SEND_CON, con);
	if (ret)
		return ret;
	udelay(5);

	ret = regmap_write(chip->regmap, S2MU004_REG_MSG_SEND_CON,
			   con | S2MU004_SEND_CON_SEND);
	if (ret)
		return ret;
	udelay(1);

	return regmap_write(chip->regmap, S2MU004_REG_MSG_SEND_CON,
			    S2MU004_SEND_CON_OP_MODE);
}

static int s2mu004_send_msg(struct s2mu004_tcpc *chip, const struct pd_message *msg)
{
	u8 buf[2 + PD_MAX_PAYLOAD * 4];
	unsigned int cnt, id;
	u16 header;
	int ret;

	header = le16_to_cpu(msg->header);
	cnt = pd_header_cnt(header);

	/* the chip keeps the MessageID counter: use its value, as vendor does */
	ret = regmap_read(chip->regmap, S2MU004_REG_ID_MONITOR, &id);
	if (ret)
		return ret;

	id = FIELD_GET(S2MU004_ID_MONITOR_MSG_ID, id);
	if (id != pd_header_msgid(header))
		dev_dbg(chip->dev, "TX MessageID %u, TCPM %u\n", id,
			pd_header_msgid(header));

	header &= ~(PD_HEADER_ID_MASK << PD_HEADER_ID_SHIFT);
	header |= id << PD_HEADER_ID_SHIFT;
	put_unaligned_le16(header, buf);
	memcpy(&buf[2], msg->payload, cnt * 4);

	dev_dbg(chip->dev, "TX header %#06x\n", header);

	ret = regmap_bulk_write(chip->regmap, S2MU004_REG_MSG_HEADER, buf, 2 + cnt * 4);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, S2MU004_REG_MSG_SEND_CON,
			   S2MU004_SEND_CON_OP_MODE | S2MU004_SEND_CON_SEND);
	if (ret)
		return ret;

	return regmap_write(chip->regmap, S2MU004_REG_MSG_SEND_CON,
			    S2MU004_SEND_CON_OP_MODE);
}

static int s2mu004_tcpm_pd_transmit(struct tcpc_dev *dev, enum tcpm_transmit_type type,
				    const struct pd_message *msg,
				    unsigned int negotiated_rev)
{
	struct s2mu004_tcpc *chip = tcpc_to_chip(dev);
	int ret;

	mutex_lock(&chip->lock);

	switch (type) {
	case TCPC_TX_SOP:
		chip->tx_pending = true;
		ret = s2mu004_send_msg(chip, msg);
		if (ret)
			chip->tx_pending = false;
		break;
	case TCPC_TX_HARD_RESET:
		chip->tx_pending = false;
		ret = s2mu004_send_hard_reset(chip);
		/* no completion interrupt for a hard reset */
		if (!ret)
			tcpm_pd_transmit_complete(chip->tcpm_port, TCPC_TX_SUCCESS);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&chip->lock);

	return ret;
}

/* VBUS sensing: the charger supplied by this port reports it */
static void s2mu004_vbus_work(struct work_struct *work)
{
	struct s2mu004_tcpc *chip = container_of(work, struct s2mu004_tcpc, vbus_work);
	union power_supply_propval val;
	struct power_supply *charger;
	struct tcpm_port *port;
	bool present;

	spin_lock_irq(&chip->charger_lock);
	charger = chip->charger;
	spin_unlock_irq(&chip->charger_lock);

	if (!charger ||
	    power_supply_get_property(charger, POWER_SUPPLY_PROP_ONLINE, &val))
		return;

	present = val.intval;
	if (present == READ_ONCE(chip->vbus_present))
		return;

	dev_dbg(chip->dev, "VBUS %s\n", present ? "on" : "off");
	WRITE_ONCE(chip->vbus_present, present);

	/* registering the port makes the charger report a change already */
	port = READ_ONCE(chip->tcpm_port);
	if (port)
		tcpm_vbus_change(port);
}

static bool s2mu004_supplies(struct s2mu004_tcpc *chip, struct power_supply *psy)
{
	struct fwnode_handle *fwnode;
	bool found = false;
	int i;

	for (i = 0; !found; i++) {
		fwnode = fwnode_find_reference(dev_fwnode(&psy->dev), "power-supplies", i);
		if (IS_ERR(fwnode))
			break;

		found = fwnode == dev_fwnode(chip->dev);
		fwnode_handle_put(fwnode);
	}

	return found;
}

static int s2mu004_psy_notifier(struct notifier_block *nb, unsigned long event,
				void *data)
{
	struct s2mu004_tcpc *chip = container_of(nb, struct s2mu004_tcpc, psy_nb);
	struct power_supply *psy = data;
	unsigned long flags;
	bool ours;

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	spin_lock_irqsave(&chip->charger_lock, flags);
	ours = psy == chip->charger;
	if (!chip->charger && s2mu004_supplies(chip, psy)) {
		get_device(&psy->dev);
		chip->charger = psy;
		ours = true;
	}
	spin_unlock_irqrestore(&chip->charger_lock, flags);

	if (ours)
		schedule_work(&chip->vbus_work);

	return NOTIFY_OK;
}

static void s2mu004_init_tcpc_dev(struct tcpc_dev *tcpc)
{
	tcpc->init = s2mu004_tcpm_init;
	tcpc->get_vbus = s2mu004_tcpm_get_vbus;
	tcpc->set_cc = s2mu004_tcpm_set_cc;
	tcpc->get_cc = s2mu004_tcpm_get_cc;
	tcpc->set_polarity = s2mu004_tcpm_set_polarity;
	tcpc->set_vconn = s2mu004_tcpm_set_vconn;
	tcpc->set_vbus = s2mu004_tcpm_set_vbus;
	tcpc->set_pd_rx = s2mu004_tcpm_set_pd_rx;
	tcpc->set_roles = s2mu004_tcpm_set_roles;
	tcpc->start_toggling = s2mu004_tcpm_start_toggling;
	tcpc->pd_transmit = s2mu004_tcpm_pd_transmit;
}

static int s2mu004_tcpc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s2mu004_tcpc *chip;
	struct tcpm_port *port;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	spin_lock_init(&chip->charger_lock);
	INIT_WORK(&chip->vbus_work, s2mu004_vbus_work);
	INIT_DELAYED_WORK(&chip->lpm_work, s2mu004_lpm_work);
	i2c_set_clientdata(client, chip);

	ret = devm_mutex_init(dev, &chip->lock);
	if (ret)
		return ret;

	chip->regmap = devm_regmap_init_i2c(client, &s2mu004_tcpc_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap), "failed to init regmap\n");

	chip->tcpc_dev.fwnode = device_get_named_child_node(dev, "connector");
	if (!chip->tcpc_dev.fwnode)
		return dev_err_probe(dev, -EINVAL, "missing connector node\n");

	s2mu004_init_tcpc_dev(&chip->tcpc_dev);

	/* before the port: its power supply lets the charger probe */
	chip->psy_nb.notifier_call = s2mu004_psy_notifier;
	ret = power_supply_reg_notifier(&chip->psy_nb);
	if (ret)
		goto err_fwnode;

	port = tcpm_register_port(dev, &chip->tcpc_dev);
	if (IS_ERR(port)) {
		ret = dev_err_probe(dev, PTR_ERR(port), "failed to register the TCPM port\n");
		goto err_notifier;
	}
	WRITE_ONCE(chip->tcpm_port, port);

	ret = request_threaded_irq(client->irq, NULL, s2mu004_tcpc_irq, IRQF_ONESHOT,
				   dev_name(dev), chip);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request the IRQ\n");
		goto err_port;
	}

	/* VBUS may have been reported before the port existed */
	tcpm_vbus_change(port);

	return 0;

err_port:
	power_supply_unreg_notifier(&chip->psy_nb);
	cancel_work_sync(&chip->vbus_work);
	tcpm_unregister_port(port);
	goto err_fwnode;
err_notifier:
	power_supply_unreg_notifier(&chip->psy_nb);
	cancel_work_sync(&chip->vbus_work);
err_fwnode:
	fwnode_handle_put(chip->tcpc_dev.fwnode);

	return ret;
}

static void s2mu004_tcpc_remove(struct i2c_client *client)
{
	struct s2mu004_tcpc *chip = i2c_get_clientdata(client);

	free_irq(client->irq, chip);
	cancel_delayed_work_sync(&chip->lpm_work);
	/* unregistering the port changes the charger: stop listening first */
	power_supply_unreg_notifier(&chip->psy_nb);
	cancel_work_sync(&chip->vbus_work);
	tcpm_unregister_port(chip->tcpm_port);

	if (chip->vbus) {
		if (chip->vbus_on)
			regulator_disable(chip->vbus);
		regulator_put(chip->vbus);
	}
	if (chip->charger)
		put_device(&chip->charger->dev);

	fwnode_handle_put(chip->tcpc_dev.fwnode);
}

static const struct of_device_id s2mu004_tcpc_of_match[] = {
	{ .compatible = "samsung,s2mu004-usbpd" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu004_tcpc_of_match);

static const struct i2c_device_id s2mu004_tcpc_i2c_id[] = {
	{ "s2mu004-usbpd" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mu004_tcpc_i2c_id);

static struct i2c_driver s2mu004_tcpc_driver = {
	.driver = {
		.name = "s2mu004-tcpc",
		.of_match_table = s2mu004_tcpc_of_match,
	},
	.probe = s2mu004_tcpc_probe,
	.remove = s2mu004_tcpc_remove,
	.id_table = s2mu004_tcpc_i2c_id,
};
module_i2c_driver(s2mu004_tcpc_driver);

MODULE_AUTHOR("Martin Štrobl <recordyletsplays@gmail.com>");
MODULE_DESCRIPTION("Samsung S2MU004 USB Type-C port controller driver");
MODULE_LICENSE("GPL");
