// SPDX-License-Identifier: GPL-2.0
/*
 * Battery Charger Driver for Samsung S2M series PMICs.
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 * Copyright (c) 2026 Kaustabh Chakraborty <kauschluss@disroot.org>
 * Copyright (c) 2026 Łukasz Lebiedziński <kernel@lvkasz.us>
 */

#include <linux/alarmtimer.h>
#include <linux/devm-helpers.h>
#include <linux/extcon.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mu004.h>
#include <linux/mfd/samsung/s2mu005.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

struct s2m_chgr {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct extcon_dev *extcon;
	struct work_struct extcon_work;
	struct notifier_block extcon_nb;
	struct delayed_work wdt_work;
	struct alarm wdt_alarm;
	struct wakeup_source *wdt_ws;
	int done_irq;

	/* S2MU004 charging policy */
	struct mutex lock;
	struct iio_channel *batt_temp;
	int batt_chg_max_ua;
	enum {
		S2MU004_TEMP_NORMAL,
		S2MU004_TEMP_WARM,
		S2MU004_TEMP_STOP,
	} temp_state;
};

/*
 * The S2MU004 charger has a watchdog which suspends charging when it is not
 * cleared in time (the bootloader leaves it enabled with an 80 s timeout).
 * Clear it well within that period. While a charger is attached, a wakeup
 * alarm schedules the clearing, so that it also happens while the system is
 * suspended; otherwise a plain delayed work does.
 */
#define S2MU004_CHGR_WDT_INTERVAL_S	30

/*
 * Charging limits per BC1.2 port type (input current, charge current, in
 * uA), from the vendor device tree of the Galaxy A3 (2017). Without a
 * detected type the input stays at the USB 2.0 default.
 */
#define S2MU004_CHGR_DEFAULT_INPUT_UA	500000
#define S2MU004_CHGR_DEFAULT_CHG_UA	500000

/*
 * Battery temperature limits (milli-degrees Celsius), also from the vendor
 * device tree: stop charging outside 0..50 C (resume inside 5..45 C) and
 * charge with a reduced current above 41 C (until below 39 C).
 */
#define S2MU004_TEMP_STOP_HOT		50000
#define S2MU004_TEMP_RESUME_HOT		45000
#define S2MU004_TEMP_WARM		41000
#define S2MU004_TEMP_RESUME_WARM	39000
#define S2MU004_TEMP_STOP_COLD		0
#define S2MU004_TEMP_RESUME_COLD	5000
#define S2MU004_CHGR_WARM_CHG_UA	825000

static void s2m_chgr_get_usb_type(struct s2m_chgr *priv, int *value)
{
	if (extcon_get_state(priv->extcon, EXTCON_CHG_USB_CDP) > 0)
		*value = POWER_SUPPLY_USB_TYPE_CDP;
	else if (extcon_get_state(priv->extcon, EXTCON_CHG_USB_SDP) > 0)
		*value = POWER_SUPPLY_USB_TYPE_SDP;
	else if (extcon_get_state(priv->extcon, EXTCON_CHG_USB_DCP) > 0)
		*value = POWER_SUPPLY_USB_TYPE_DCP;
	else
		*value = POWER_SUPPLY_USB_TYPE_UNKNOWN;
}


static int s2mu004_chgr_get_online(struct s2m_chgr *priv, int *value)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_STATUS0, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(S2MU004_CHGR_CHGIN_STS, val)) {
	case S2MU004_CHGR_CHGIN_STS_VALID:
	case S2MU004_CHGR_CHGIN_STS_VALID_ALT:
		*value = 1;
		break;
	default:
		*value = 0;
	}

	return 0;
}

static int s2mu004_chgr_get_status(struct s2m_chgr *priv, int *value)
{
	u32 val, ctrl0;
	int online, ret;

	ret = s2mu004_chgr_get_online(priv, &online);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_STATUS1, &val);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_CTRL0, &ctrl0);
	if (ret)
		return ret;

	if (!online || (val & S2MU004_CHGR_SELF_DISCHG)) {
		*value = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (FIELD_GET(S2MU004_CHGR_OP_MODE, ctrl0) != S2MU004_CHGR_OP_MODE_CHG) {
		/* the input powers the system only (buck mode) */
		*value = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else if (val & S2MU004_CHGR_DONE) {
		*value = POWER_SUPPLY_STATUS_FULL;
	} else {
		switch (FIELD_GET(S2MU004_CHGR_FAULT, val)) {
		case 0:
		case S2MU004_CHGR_FAULT_PRECHG:
		case S2MU004_CHGR_FAULT_FASTCHG:
			*value = POWER_SUPPLY_STATUS_CHARGING;
			break;
		default:
			*value = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	return 0;
}

static int s2mu004_chgr_get_health(struct s2m_chgr *priv, int *value)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_STATUS1, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(S2MU004_CHGR_FAULT, val)) {
	case S2MU004_CHGR_FAULT_WD_SUSPEND:
	case S2MU004_CHGR_FAULT_WD_RESET:
		*value = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
		break;
	case S2MU004_CHGR_FAULT_TSD:
		*value = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case S2MU004_CHGR_FAULT_TFB:
		*value = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		break;
	default:
		*value = POWER_SUPPLY_HEALTH_GOOD;
	}

	return 0;
}

/* CTRL2: input current limit, 25 mA steps from 100 mA (code 2) to 2500 mA */
static int s2mu004_chgr_get_input_limit(struct s2m_chgr *priv, int *ua)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_CTRL2, &val);
	if (ret)
		return ret;

	*ua = (FIELD_GET(S2MU004_CHGR_INPUT_CURR_LIM, val) * 25 + 50) * 1000;

	return 0;
}

static int s2mu004_chgr_set_input_limit(struct s2m_chgr *priv, int ua)
{
	u32 code = clamp((ua / 1000 - 50) / 25, 2, 0x62);

	return regmap_update_bits(priv->regmap, S2MU004_REG_CHGR_CTRL2,
				  S2MU004_CHGR_INPUT_CURR_LIM,
				  FIELD_PREP(S2MU004_CHGR_INPUT_CURR_LIM, code));
}

/* CTRL9: fast charge current, 25 mA steps from 100 mA (code 3) to 3150 mA */
static int s2mu004_chgr_get_charge_current(struct s2m_chgr *priv, int *ua)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_CTRL9, &val);
	if (ret)
		return ret;

	*ua = (FIELD_GET(S2MU004_CHGR_FAST_CHG_CURR, val) + 1) * 25 * 1000;

	return 0;
}

static int s2mu004_chgr_set_charge_current(struct s2m_chgr *priv, int ua)
{
	u32 code = clamp(ua / 1000 / 25 - 1, 3, 0x7d);

	return regmap_update_bits(priv->regmap, S2MU004_REG_CHGR_CTRL9,
				  S2MU004_CHGR_FAST_CHG_CURR,
				  FIELD_PREP(S2MU004_CHGR_FAST_CHG_CURR, code));
}

/* CTRL6: float voltage, 10 mV steps from 3900 mV */
static int s2mu004_chgr_get_float_voltage(struct s2m_chgr *priv, int *uv)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_CTRL6, &val);
	if (ret)
		return ret;

	*uv = (3900 + FIELD_GET(S2MU004_CHGR_VF_VBAT, val) * 10) * 1000;

	return 0;
}

/* CTRL11: first top-off (termination) current, 25 mA steps from 100 mA */
static int s2mu004_chgr_get_term_current(struct s2m_chgr *priv, int *ua)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_CTRL11, &val);
	if (ret)
		return ret;

	*ua = (100 + FIELD_GET(S2MU004_CHGR_TOPOFF_CURR1, val) * 25) * 1000;

	return 0;
}

static int s2mu004_chgr_set_mode(struct s2m_chgr *priv, u32 mode)
{
	return regmap_update_bits(priv->regmap, S2MU004_REG_CHGR_CTRL0,
				  S2MU004_CHGR_OP_MODE,
				  FIELD_PREP(S2MU004_CHGR_OP_MODE, mode));
}

/*
 * Program the input and charge current for the detected port type and the
 * battery temperature, and stop or resume charging. Called with priv->lock
 * held.
 */
static void s2mu004_chgr_apply(struct s2m_chgr *priv)
{
	int input_ua = S2MU004_CHGR_DEFAULT_INPUT_UA;
	int chg_ua = S2MU004_CHGR_DEFAULT_CHG_UA;
	int type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	u32 ctrl0;
	int ret;

	if (priv->extcon)
		s2m_chgr_get_usb_type(priv, &type);

	switch (type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		input_ua = 500000;
		chg_ua = 800000;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		input_ua = 1500000;
		chg_ua = 1500000;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		input_ua = 900000;
		chg_ua = 1200000;
		break;
	default:
		break;
	}

	if (priv->batt_chg_max_ua > 0)
		chg_ua = min(chg_ua, priv->batt_chg_max_ua);
	if (priv->temp_state == S2MU004_TEMP_WARM)
		chg_ua = min(chg_ua, S2MU004_CHGR_WARM_CHG_UA);

	ret = s2mu004_chgr_set_input_limit(priv, input_ua);
	if (!ret)
		ret = s2mu004_chgr_set_charge_current(priv, chg_ua);
	if (ret)
		dev_err(priv->dev, "failed to set charging currents (%d)\n", ret);

	if (regmap_read(priv->regmap, S2MU004_REG_CHGR_CTRL0, &ctrl0))
		return;

	/* Only switch between buck and charge mode, never out of others */
	switch (FIELD_GET(S2MU004_CHGR_OP_MODE, ctrl0)) {
	case S2MU004_CHGR_OP_MODE_CHG:
		if (priv->temp_state == S2MU004_TEMP_STOP)
			s2mu004_chgr_set_mode(priv, S2MU004_CHGR_OP_MODE_BUCK);
		break;
	case S2MU004_CHGR_OP_MODE_BUCK:
		if (priv->temp_state != S2MU004_TEMP_STOP)
			s2mu004_chgr_set_mode(priv, S2MU004_CHGR_OP_MODE_CHG);
		break;
	default:
		break;
	}
}

/* Update the battery temperature state, true if it changed */
static bool s2mu004_chgr_update_temp(struct s2m_chgr *priv)
{
	int old = priv->temp_state, new = old;
	int temp;

	if (!priv->batt_temp || iio_read_channel_processed(priv->batt_temp, &temp) < 0)
		return false;

	switch (old) {
	case S2MU004_TEMP_NORMAL:
		if (temp >= S2MU004_TEMP_STOP_HOT || temp <= S2MU004_TEMP_STOP_COLD)
			new = S2MU004_TEMP_STOP;
		else if (temp >= S2MU004_TEMP_WARM)
			new = S2MU004_TEMP_WARM;
		break;
	case S2MU004_TEMP_WARM:
		if (temp >= S2MU004_TEMP_STOP_HOT)
			new = S2MU004_TEMP_STOP;
		else if (temp < S2MU004_TEMP_RESUME_WARM)
			new = S2MU004_TEMP_NORMAL;
		break;
	case S2MU004_TEMP_STOP:
		if (temp < S2MU004_TEMP_RESUME_HOT && temp > S2MU004_TEMP_RESUME_COLD)
			new = temp >= S2MU004_TEMP_WARM ? S2MU004_TEMP_WARM :
							  S2MU004_TEMP_NORMAL;
		break;
	}

	if (new == old)
		return false;

	dev_info(priv->dev, "battery at %d.%d C: %s\n", temp / 1000,
		 abs(temp % 1000) / 100,
		 new == S2MU004_TEMP_STOP ? "charging stopped" :
		 new == S2MU004_TEMP_WARM ? "reduced charging current" :
					    "normal charging");
	priv->temp_state = new;

	return true;
}

static int s2mu004_chgr_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct s2m_chgr *priv = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return s2mu004_chgr_get_online(priv, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return s2mu004_chgr_get_status(priv, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return s2mu004_chgr_get_health(priv, &val->intval);
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		if (priv->extcon)
			s2m_chgr_get_usb_type(priv, &val->intval);
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return s2mu004_chgr_get_input_limit(priv, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return s2mu004_chgr_get_charge_current(priv, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return s2mu004_chgr_get_float_voltage(priv, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return s2mu004_chgr_get_term_current(priv, &val->intval);
	default:
		return -EINVAL;
	}
}

static void s2mu004_chgr_wdt_work(struct work_struct *work)
{
	struct s2m_chgr *priv = container_of(to_delayed_work(work),
					     struct s2m_chgr, wdt_work);
	u32 ctrl0, sts1;
	int online, ret;

	mutex_lock(&priv->lock);
	if (s2mu004_chgr_update_temp(priv)) {
		s2mu004_chgr_apply(priv);
		power_supply_changed(priv->psy);
	}
	mutex_unlock(&priv->lock);

	ret = regmap_set_bits(priv->regmap, S2MU004_REG_CHGR_CTRL14,
			      S2MU004_CHGR_WDT_CLR);
	if (ret) {
		dev_err(priv->dev, "failed to clear watchdog (%d)\n", ret);
		goto out;
	}

	if (regmap_read(priv->regmap, S2MU004_REG_CHGR_STATUS1, &sts1) ||
	    regmap_read(priv->regmap, S2MU004_REG_CHGR_CTRL0, &ctrl0) ||
	    s2mu004_chgr_get_online(priv, &online))
		goto out;

	switch (FIELD_GET(S2MU004_CHGR_FAULT, sts1)) {
	case S2MU004_CHGR_FAULT_WD_SUSPEND:
	case S2MU004_CHGR_FAULT_WD_RESET:
		break;
	default:
		goto out;
	}

	/* Only resume charging that was enabled and has a supply to run from */
	if (!online || !(ctrl0 & S2MU004_CHGR_EN_CHG) ||
	    FIELD_GET(S2MU004_CHGR_OP_MODE, ctrl0) != S2MU004_CHGR_OP_MODE_CHG)
		goto out;

	dev_info(priv->dev, "watchdog suspended charging, restarting it\n");

	ret = regmap_update_bits(priv->regmap, S2MU004_REG_CHGR_CTRL0,
				 S2MU004_CHGR_OP_MODE,
				 FIELD_PREP(S2MU004_CHGR_OP_MODE,
					    S2MU004_CHGR_OP_MODE_BUCK));
	if (ret)
		goto out;
	msleep(100);
	ret = regmap_update_bits(priv->regmap, S2MU004_REG_CHGR_CTRL0,
				 S2MU004_CHGR_OP_MODE,
				 FIELD_PREP(S2MU004_CHGR_OP_MODE,
					    S2MU004_CHGR_OP_MODE_CHG));
	if (ret)
		dev_err(priv->dev, "failed to restore charging mode (%d)\n", ret);

	power_supply_changed(priv->psy);
out:
	if (s2mu004_chgr_get_online(priv, &online))
		online = 0;

	if (online) {
		alarm_start_timer(&priv->wdt_alarm,
				  ktime_set(S2MU004_CHGR_WDT_INTERVAL_S, 0), true);
	} else {
		alarm_try_to_cancel(&priv->wdt_alarm);
		queue_delayed_work(system_freezable_wq, &priv->wdt_work,
				   S2MU004_CHGR_WDT_INTERVAL_S * HZ);
	}

	__pm_relax(priv->wdt_ws);
}

static void s2mu004_chgr_wdt_alarm(struct alarm *alarm, ktime_t now)
{
	struct s2m_chgr *priv = container_of(alarm, struct s2m_chgr, wdt_alarm);

	/* Stay awake until the work has cleared the watchdog */
	__pm_stay_awake(priv->wdt_ws);
	mod_delayed_work(system_freezable_wq, &priv->wdt_work, 0);
}

static void s2mu004_chgr_wdt_stop(void *data)
{
	struct s2m_chgr *priv = data;

	/* A running work may arm the alarm again: cancel both, then once more */
	alarm_cancel(&priv->wdt_alarm);
	cancel_delayed_work_sync(&priv->wdt_work);
	alarm_cancel(&priv->wdt_alarm);
	wakeup_source_unregister(priv->wdt_ws);
}

static int s2mu005_chgr_get_online(struct s2m_chgr *priv, int *value)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU005_REG_CHGR_STATUS0, &val);
	if (ret) {
		dev_err(priv->dev, "failed to read register (%d)\n", ret);
		return ret;
	}

	*value = !!(val & S2MU005_CHGR_CHG);

	return 0;
}

static int s2mu005_chgr_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct s2m_chgr *priv = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = s2mu005_chgr_get_online(priv, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		s2m_chgr_get_usb_type(priv, &val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s2mu005_chgr_mode_set_host(struct s2m_chgr *priv)
{
	int ret;

	/* set mode to OTG */
	ret = regmap_update_bits(priv->regmap, S2MU005_REG_CHGR_CTRL0,
				 S2MU005_CHGR_OP_MODE,
				 FIELD_PREP(S2MU005_CHGR_OP_MODE,
					    S2MU005_CHGR_OP_MODE_OTG));
	if (ret) {
		dev_err(priv->dev, "failed to set OTG mode (%d)\n", ret);
		return ret;
	}

	/* set boost frequency to 2MHz */
	ret = regmap_update_bits(priv->regmap, S2MU005_REG_CHGR_CTRL11,
				 S2MU005_CHGR_OSC_BOOST,
				 FIELD_PREP(S2MU005_CHGR_OSC_BOOST,
					    S2MU005_CHGR_OSC_BOOST_2MHZ));
	if (ret) {
		dev_err(priv->dev, "failed to set boost frequency (%d)\n", ret);
		return ret;
	}

	/* set OTG current limit to 1.5 A */
	ret = regmap_update_bits(priv->regmap, S2MU005_REG_CHGR_CTRL4,
				 S2MU005_CHGR_OTG_OCP,
				 FIELD_PREP(S2MU005_CHGR_OTG_OCP,
					    S2MU005_CHGR_OTG_OCP_1P5A));
	if (ret) {
		dev_err(priv->dev, "failed to set OTG current limit (%d)\n", ret);
		return ret;
	}

	/* VBUS switches are OFF when OTG over-current happens */
	ret = regmap_set_bits(priv->regmap, S2MU005_REG_CHGR_CTRL4,
			      S2MU005_CHGR_OTG_OCP_OFF);
	if (ret) {
		dev_err(priv->dev, "failed to set OTG OCP switch (%d)\n", ret);
		return ret;
	}

	/* set OTG voltage to 5.1 V */
	ret = regmap_update_bits(priv->regmap, S2MU005_REG_CHGR_CTRL5,
				 S2MU005_CHGR_VMID_BOOST,
				 FIELD_PREP(S2MU005_CHGR_VMID_BOOST,
					    S2MU005_CHGR_VMID_BOOST_5P1V));
	if (ret) {
		dev_err(priv->dev, "failed to set OTG voltage (%d)\n", ret);
		return ret;
	}

	/* turn on OTG */
	ret = regmap_update_bits(priv->regmap, S2MU005_REG_CHGR_CTRL15,
				 S2MU005_CHGR_OTG_EN,
				 FIELD_PREP(S2MU005_CHGR_OTG_EN,
					    S2MU005_CHGR_OTG_EN_ON));
	if (ret) {
		dev_err(priv->dev, "failed to turn on OTG (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int s2mu005_chgr_mode_set_charger(struct s2m_chgr *priv)
{
	int ret;

	/* first reset to mode 0 */
	ret = regmap_clear_bits(priv->regmap, S2MU005_REG_CHGR_CTRL0,
				S2MU005_CHGR_OP_MODE);
	if (ret) {
		dev_err(priv->dev, "failed to reset opmode (%d)\n", ret);
		return ret;
	}

	/* wait for the charger to settle before switching to charging mode */
	msleep(50);
	/* then set to charging mode */
	ret = regmap_update_bits(priv->regmap, S2MU005_REG_CHGR_CTRL0,
				 S2MU005_CHGR_OP_MODE,
				 FIELD_PREP(S2MU005_CHGR_OP_MODE,
					    S2MU005_CHGR_OP_MODE_CHG));
	if (ret) {
		dev_err(priv->dev, "failed to set opmode to charging (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int s2mu005_chgr_mode_unset(struct s2m_chgr *priv)
{
	int ret;

	/* turn off OTG */
	ret = regmap_clear_bits(priv->regmap, S2MU005_REG_CHGR_CTRL15,
				S2MU005_CHGR_OTG_EN);
	if (ret) {
		dev_err(priv->dev, "failed to turn off OTG (%d)\n", ret);
		return ret;
	}

	/* reset operation mode */
	ret = regmap_clear_bits(priv->regmap, S2MU005_REG_CHGR_CTRL0,
				S2MU005_CHGR_OP_MODE);
	if (ret) {
		dev_err(priv->dev, "failed to reset opmode (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void s2mu005_chgr_extcon_work(struct work_struct *work)
{
	struct s2m_chgr *priv = container_of(work, struct s2m_chgr, extcon_work);

	if (extcon_get_state(priv->extcon, EXTCON_USB_HOST) > 0)
		s2mu005_chgr_mode_set_host(priv);
	else if (extcon_get_state(priv->extcon, EXTCON_USB) > 0)
		s2mu005_chgr_mode_set_charger(priv);
	else
		s2mu005_chgr_mode_unset(priv);

	power_supply_changed(priv->psy);
}

static void s2mu004_chgr_extcon_work(struct work_struct *work)
{
	struct s2m_chgr *priv = container_of(work, struct s2m_chgr, extcon_work);

	mutex_lock(&priv->lock);
	s2mu004_chgr_apply(priv);
	mutex_unlock(&priv->lock);

	power_supply_changed(priv->psy);
}

static const enum power_supply_property s2mu004_chgr_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static const struct power_supply_desc s2mu004_chgr_psy_desc = {
	.name = "s2mu004-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = s2mu004_chgr_properties,
	.num_properties = ARRAY_SIZE(s2mu004_chgr_properties),
	.get_property = s2mu004_chgr_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
};

static const enum power_supply_property s2mu005_chgr_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc s2mu005_chgr_psy_desc = {
	.name = "s2mu005-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = s2mu005_chgr_properties,
	.num_properties = ARRAY_SIZE(s2mu005_chgr_properties),
	.get_property = s2mu005_chgr_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
};

static int s2m_chgr_extcon_notifier(struct notifier_block *nb,
					unsigned long event, void *param)
{
	struct s2m_chgr *priv = container_of(nb, struct s2m_chgr, extcon_nb);

	schedule_work(&priv->extcon_work);

	return NOTIFY_OK;
}

static irqreturn_t s2mu004_chgr_irq(int irq, void *data)
{
	struct s2m_chgr *priv = data;

	/* Cable or charge state changed: re-check the watchdog right away */
	mod_delayed_work(system_freezable_wq, &priv->wdt_work, 0);
	power_supply_changed(priv->psy);

	return IRQ_HANDLED;
}

static int s2mu004_chgr_probe(struct platform_device *pdev, struct s2m_chgr *priv)
{
	static const char * const irq_names[] = {
		"chgin", "done", "fault", "vbus-on", "vbus-off",
	};
	struct device_node *muic_node __free(device_node) = NULL;
	struct power_supply_battery_info *info;
	struct power_supply_config psy_cfg = {};
	struct device *dev = priv->dev;
	int i, irq, ret;

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	/* The MUIC (BC1.2 port type) is optional */
	muic_node = of_get_child_by_name(dev->parent->of_node, "muic");
	if (muic_node) {
		priv->extcon = extcon_find_edev_by_node(muic_node);
		if (IS_ERR(priv->extcon))
			return -EPROBE_DEFER;
	}

	psy_cfg.drv_data = priv;
	psy_cfg.fwnode = dev_fwnode(dev->parent);
	priv->psy = devm_power_supply_register(dev, &s2mu004_chgr_psy_desc, &psy_cfg);
	if (IS_ERR(priv->psy))
		return dev_err_probe(dev, PTR_ERR(priv->psy),
				     "failed to register power supply subsystem\n");

	/* Battery temperature for the charging limits, optional */
	priv->batt_temp = devm_fwnode_iio_channel_get_by_name(dev, dev_fwnode(dev->parent),
								 "batt-temp");
	if (IS_ERR(priv->batt_temp)) {
		if (PTR_ERR(priv->batt_temp) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		priv->batt_temp = NULL;
	}

	if (!power_supply_get_battery_info(priv->psy, &info))
		priv->batt_chg_max_ua = info->constant_charge_current_max_ua;

	ret = devm_work_autocancel(dev, &priv->extcon_work, s2mu004_chgr_extcon_work);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize extcon work\n");

	if (priv->extcon) {
		priv->extcon_nb.notifier_call = s2m_chgr_extcon_notifier;
		ret = devm_extcon_register_notifier_all(dev, priv->extcon,
							&priv->extcon_nb);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to register extcon notifier\n");
	}

	mutex_lock(&priv->lock);
	s2mu004_chgr_update_temp(priv);
	s2mu004_chgr_apply(priv);
	mutex_unlock(&priv->lock);

	ret = devm_delayed_work_autocancel(dev, &priv->wdt_work,
					   s2mu004_chgr_wdt_work);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize watchdog work\n");

	priv->wdt_ws = wakeup_source_register(dev, "s2mu004-charger-wdt");
	if (!priv->wdt_ws)
		return dev_err_probe(dev, -ENOMEM, "failed to register wakeup source\n");
	alarm_init(&priv->wdt_alarm, ALARM_BOOTTIME, s2mu004_chgr_wdt_alarm);
	/* After the work's autocancel action: runs before it on removal */
	ret = devm_add_action_or_reset(dev, s2mu004_chgr_wdt_stop, priv);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(irq_names); i++) {
		irq = platform_get_irq_byname_optional(pdev, irq_names[i]);
		if (irq < 0)
			continue;	/* no PMIC interrupt: values are still read on demand */

		ret = devm_request_threaded_irq(dev, irq, NULL, s2mu004_chgr_irq,
						IRQF_ONESHOT, irq_names[i], priv);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request %s IRQ\n",
					     irq_names[i]);
		if (!strcmp(irq_names[i], "done"))
			priv->done_irq = irq;
	}

	queue_delayed_work(system_freezable_wq, &priv->wdt_work, 0);

	return 0;
}

static int s2m_chgr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sec_pmic_dev *pmic_drvdata = dev_get_drvdata(dev->parent);
	struct s2m_chgr *priv;
	struct device_node *extcon_node __free(device_node) = NULL;
	struct power_supply_config psy_cfg = {};
	const struct power_supply_desc *psy_desc;
	work_func_t extcon_work_func;
	int ret;

	priv = devm_kzalloc_obj(dev, *priv);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = dev;
	priv->regmap = pmic_drvdata->regmap_pmic;

	switch (platform_get_device_id(pdev)->driver_data) {
	case S2MU004:
		return s2mu004_chgr_probe(pdev, priv);
	case S2MU005:
		psy_desc = &s2mu005_chgr_psy_desc;
		extcon_work_func = s2mu005_chgr_extcon_work;
		break;
	default:
		return dev_err_probe(dev, -ENODEV,
				     "device type %d is not supported by driver\n",
				     pmic_drvdata->device_type);
	}

	/* MUIC is mandatory. If unavailable, request probe deferral */
	extcon_node = of_get_child_by_name(dev->parent->of_node, "muic");
	if (!extcon_node)
		return dev_err_probe(dev, -ENODEV, "MUIC node required but not found\n");

	priv->extcon = extcon_find_edev_by_node(extcon_node);
	if (IS_ERR(priv->extcon))
		return -EPROBE_DEFER;

	psy_cfg.drv_data = priv;
	psy_cfg.fwnode = dev_fwnode(dev->parent);
	priv->psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
	if (IS_ERR(priv->psy))
		return dev_err_probe(dev, PTR_ERR(priv->psy),
				     "failed to register power supply subsystem\n");

	ret = devm_work_autocancel(dev, &priv->extcon_work, extcon_work_func);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize extcon work\n");

	priv->extcon_nb.notifier_call = s2m_chgr_extcon_notifier;
	ret = devm_extcon_register_notifier_all(dev, priv->extcon, &priv->extcon_nb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register extcon notifier\n");

	return 0;
}

static const struct platform_device_id s2m_chgr_id_table[] = {
	{ "s2mu004-charger", S2MU004 },
	{ "s2mu005-charger", S2MU005 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, s2m_chgr_id_table);

/*
 * Charge done toggles every few seconds while a full battery sits on a weak
 * charger. Keep it from waking the system: cable and fault events still do
 * when the PMIC is a wakeup source.
 */
static int s2m_chgr_suspend(struct device *dev)
{
	struct s2m_chgr *priv = dev_get_drvdata(dev);

	if (priv->done_irq > 0)
		disable_irq(priv->done_irq);

	return 0;
}

static int s2m_chgr_resume(struct device *dev)
{
	struct s2m_chgr *priv = dev_get_drvdata(dev);

	if (priv->done_irq > 0)
		enable_irq(priv->done_irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s2m_chgr_pm_ops, s2m_chgr_suspend, s2m_chgr_resume);

static struct platform_driver s2m_chgr_driver = {
	.driver = {
		.name = "s2m-charger",
		.pm = pm_sleep_ptr(&s2m_chgr_pm_ops),
	},
	.probe = s2m_chgr_probe,
	.id_table = s2m_chgr_id_table,
};
module_platform_driver(s2m_chgr_driver);

MODULE_DESCRIPTION("Battery Charger Driver For Samsung S2M Series PMICs");
MODULE_AUTHOR("Kaustabh Chakraborty <kauschluss@disroot.org>");
MODULE_AUTHOR("Łukasz Lebiedziński <kernel@lvkasz.us>");
MODULE_LICENSE("GPL");
