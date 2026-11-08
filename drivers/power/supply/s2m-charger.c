// SPDX-License-Identifier: GPL-2.0
/*
 * Battery Charger Driver for Samsung S2M series PMICs.
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 * Copyright (c) 2026 Kaustabh Chakraborty <kauschluss@disroot.org>
 * Copyright (c) 2026 Łukasz Lebiedziński <kernel@lvkasz.us>
 */

#include <linux/devm-helpers.h>
#include <linux/extcon.h>
#include <linux/interrupt.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mu004.h>
#include <linux/mfd/samsung/s2mu005.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
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
};

/*
 * The S2MU004 charger has a watchdog which suspends charging when it is not
 * cleared in time (the bootloader leaves it enabled with an 80 s timeout).
 * Clear it well within that period.
 */
#define S2MU004_CHGR_WDT_INTERVAL	(30 * HZ)

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
	u32 val;
	int online, ret;

	ret = s2mu004_chgr_get_online(priv, &online);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, S2MU004_REG_CHGR_STATUS1, &val);
	if (ret)
		return ret;

	if (!online || (val & S2MU004_CHGR_SELF_DISCHG)) {
		*value = POWER_SUPPLY_STATUS_DISCHARGING;
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
	queue_delayed_work(system_dfl_wq, &priv->wdt_work, S2MU004_CHGR_WDT_INTERVAL);
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

static void s2mu005_chgr_get_usb_type(struct s2m_chgr *priv, int *value)
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
		s2mu005_chgr_get_usb_type(priv, &val->intval);
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

static const enum power_supply_property s2mu004_chgr_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
};

static const struct power_supply_desc s2mu004_chgr_psy_desc = {
	.name = "s2mu004-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = s2mu004_chgr_properties,
	.num_properties = ARRAY_SIZE(s2mu004_chgr_properties),
	.get_property = s2mu004_chgr_get_property,
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
	mod_delayed_work(system_dfl_wq, &priv->wdt_work, 0);
	power_supply_changed(priv->psy);

	return IRQ_HANDLED;
}

static int s2mu004_chgr_probe(struct platform_device *pdev, struct s2m_chgr *priv)
{
	static const char * const irq_names[] = {
		"chgin", "done", "fault", "vbus-on", "vbus-off",
	};
	struct power_supply_config psy_cfg = {};
	struct device *dev = priv->dev;
	int i, irq, ret;

	psy_cfg.drv_data = priv;
	psy_cfg.fwnode = dev_fwnode(dev->parent);
	priv->psy = devm_power_supply_register(dev, &s2mu004_chgr_psy_desc, &psy_cfg);
	if (IS_ERR(priv->psy))
		return dev_err_probe(dev, PTR_ERR(priv->psy),
				     "failed to register power supply subsystem\n");

	ret = devm_delayed_work_autocancel(dev, &priv->wdt_work,
					   s2mu004_chgr_wdt_work);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize watchdog work\n");

	for (i = 0; i < ARRAY_SIZE(irq_names); i++) {
		irq = platform_get_irq_byname_optional(pdev, irq_names[i]);
		if (irq < 0)
			continue;	/* no PMIC interrupt: values are still read on demand */

		ret = devm_request_threaded_irq(dev, irq, NULL, s2mu004_chgr_irq,
						IRQF_ONESHOT, irq_names[i], priv);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request %s IRQ\n",
					     irq_names[i]);
	}

	queue_delayed_work(system_dfl_wq, &priv->wdt_work, 0);

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

static struct platform_driver s2m_chgr_driver = {
	.driver = {
		.name = "s2m-charger",
	},
	.probe = s2m_chgr_probe,
	.id_table = s2m_chgr_id_table,
};
module_platform_driver(s2m_chgr_driver);

MODULE_DESCRIPTION("Battery Charger Driver For Samsung S2M Series PMICs");
MODULE_AUTHOR("Kaustabh Chakraborty <kauschluss@disroot.org>");
MODULE_AUTHOR("Łukasz Lebiedziński <kernel@lvkasz.us>");
MODULE_LICENSE("GPL");
