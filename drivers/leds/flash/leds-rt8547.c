// SPDX-License-Identifier: GPL-2.0-only
/*
 * Richtek RT8547 flash LED driver
 *
 * The RT8547 is programmed through a one-wire interface on its FLSET pin.
 * It uses the ExpressWire bit encoding with a 19-bit frame: the device
 * address, a 3-bit register address and an 8-bit value. Holding FLSET low
 * for 1.5 ms shuts the chip down and resets its registers. FLEN switches
 * the LED on in the mode selected by the registers.
 */

#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/led-class-flash.h>
#include <linux/leds-expresswire.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>

#include <media/v4l2-flash-led-class.h>

#define RT8547_DEV_ADDR			0x99
#define RT8547_FRAME_BITS		19

#define RT8547_REG_LVP			0x1
#define RT8547_REG_STROBE		0x2
#define RT8547_REG_MODE			0x3
#define RT8547_REG_TIMEOUT		0x4

/* Low input voltage protection at 3.0 V (the reset value is 3.6 V) */
#define RT8547_LVP_3_0V			0x0
/* Strobe timer only runs above this LED current: 400 mA */
#define RT8547_STROBE_TIMEI_400MA	(0x6 << 5)
#define RT8547_MODE_TORCH		BIT(4)

#define RT8547_TORCH_MIN_UA		25000
#define RT8547_TORCH_STEP_UA		25000
#define RT8547_TORCH_MAX_UA		400000
#define RT8547_FLASH_MIN_UA		100000
#define RT8547_FLASH_STEP_UA		50000
#define RT8547_FLASH_MAX_UA		1600000
#define RT8547_TIMEOUT_MIN_US		64000
#define RT8547_TIMEOUT_STEP_US		32000
#define RT8547_TIMEOUT_MAX_US		1216000

static const struct expresswire_timing rt8547_timing = {
	/* FLSET low for at least 1.5 ms shuts the chip down */
	.poweroff_us = 2000,
	/* FLSET high for at least 10 us before the first falling edge */
	.data_start_us = 10,
	.end_of_data_low_us = 10,
	.end_of_data_high_us = 100,
	/* 4 us to 180 us, the long phase more than twice the short one */
	.short_bitset_us = 10,
	.long_bitset_us = 40,
};

struct rt8547 {
	struct device *dev;
	struct led_classdev_flash fled;
	struct v4l2_flash *v4l2_flash;
	struct expresswire_common_props props;
	struct gpio_desc *flen_gpio;
	struct timer_list strobe_timer;
	/* serialises one-wire frames and the torch state */
	struct mutex lock;
	bool torch_on;
};

static struct rt8547 *to_rt8547(struct led_classdev_flash *fled)
{
	return container_of(fled, struct rt8547, fled);
}

static void rt8547_write(struct rt8547 *rt, u8 reg, u8 val)
{
	expresswire_write_bits(&rt->props,
			       RT8547_DEV_ADDR << 11 | reg << 8 | val,
			       RT8547_FRAME_BITS);
}

/* Called from the strobe timer as well: must not sleep */
static void rt8547_off(struct rt8547 *rt)
{
	gpiod_set_value(rt->flen_gpio, 0);
	gpiod_set_value(rt->props.ctrl_gpio, 0);
}

/* Shut the chip down (resetting its registers) and wake it up again */
static void rt8547_reset(struct rt8547 *rt)
{
	timer_delete_sync(&rt->strobe_timer);
	gpiod_set_value(rt->flen_gpio, 0);
	expresswire_power_off(&rt->props);
	rt8547_write(rt, RT8547_REG_LVP, RT8547_LVP_3_0V);
}

static u8 rt8547_flash_code(struct rt8547 *rt)
{
	u32 ua = rt->fled.brightness.val;

	return RT8547_STROBE_TIMEI_400MA |
	       (ua - RT8547_FLASH_MIN_UA) / RT8547_FLASH_STEP_UA;
}

static int rt8547_brightness_set(struct led_classdev *led,
				 enum led_brightness brightness)
{
	struct rt8547 *rt = to_rt8547(lcdev_to_flcdev(led));
	u8 mode;

	guard(mutex)(&rt->lock);

	if (brightness == LED_OFF) {
		timer_delete_sync(&rt->strobe_timer);
		rt8547_off(rt);
		rt->torch_on = false;
		return 0;
	}

	mode = RT8547_MODE_TORCH | (brightness - 1);
	if (rt->torch_on) {
		rt8547_write(rt, RT8547_REG_MODE, mode);
		return 0;
	}

	rt8547_reset(rt);
	rt8547_write(rt, RT8547_REG_STROBE, rt8547_flash_code(rt));
	rt8547_write(rt, RT8547_REG_MODE, mode);
	gpiod_set_value(rt->flen_gpio, 1);
	rt->torch_on = true;

	return 0;
}

static int rt8547_flash_brightness_set(struct led_classdev_flash *fled,
				       u32 brightness)
{
	/* Stored by the LED flash core, applied at the next strobe */
	return 0;
}

static int rt8547_flash_timeout_set(struct led_classdev_flash *fled,
				    u32 timeout)
{
	/* Stored by the LED flash core, applied at the next strobe */
	return 0;
}

static int rt8547_flash_strobe_set(struct led_classdev_flash *fled, bool state)
{
	struct rt8547 *rt = to_rt8547(fled);
	u32 timeout = fled->timeout.val;

	guard(mutex)(&rt->lock);

	rt->torch_on = false;
	fled->led_cdev.brightness = LED_OFF;

	if (!state) {
		timer_delete_sync(&rt->strobe_timer);
		rt8547_off(rt);
		return 0;
	}

	rt8547_reset(rt);
	rt8547_write(rt, RT8547_REG_STROBE, rt8547_flash_code(rt));
	rt8547_write(rt, RT8547_REG_MODE, 0);
	rt8547_write(rt, RT8547_REG_TIMEOUT,
		     (timeout - RT8547_TIMEOUT_MIN_US) / RT8547_TIMEOUT_STEP_US);
	gpiod_set_value(rt->flen_gpio, 1);
	mod_timer(&rt->strobe_timer, jiffies + usecs_to_jiffies(timeout));

	return 0;
}

static int rt8547_flash_strobe_get(struct led_classdev_flash *fled,
				   bool *state)
{
	struct rt8547 *rt = to_rt8547(fled);

	*state = timer_pending(&rt->strobe_timer);

	return 0;
}

static const struct led_flash_ops rt8547_flash_ops = {
	.flash_brightness_set = rt8547_flash_brightness_set,
	.strobe_set = rt8547_flash_strobe_set,
	.strobe_get = rt8547_flash_strobe_get,
	.timeout_set = rt8547_flash_timeout_set,
};

static void rt8547_strobe_timeout(struct timer_list *t)
{
	struct rt8547 *rt = timer_container_of(rt, t, strobe_timer);

	/* The chip has ended the strobe itself: shut it down */
	rt8547_off(rt);
}

static void rt8547_disable(void *data)
{
	struct rt8547 *rt = data;

	timer_delete_sync(&rt->strobe_timer);
	rt8547_off(rt);
}

static void rt8547_v4l2_flash_release(void *data)
{
	v4l2_flash_release(data);
}

static int rt8547_parse_led(struct rt8547 *rt, struct fwnode_handle *child,
			    u32 *torch_ua, u32 *flash_ua, u32 *timeout_us)
{
	struct device *dev = rt->dev;
	int ret;

	ret = fwnode_property_read_u32(child, "led-max-microamp", torch_ua);
	if (ret)
		return dev_err_probe(dev, ret, "missing led-max-microamp\n");

	ret = fwnode_property_read_u32(child, "flash-max-microamp", flash_ua);
	if (ret)
		return dev_err_probe(dev, ret, "missing flash-max-microamp\n");

	ret = fwnode_property_read_u32(child, "flash-max-timeout-us",
				       timeout_us);
	if (ret)
		return dev_err_probe(dev, ret, "missing flash-max-timeout-us\n");

	if (*torch_ua < RT8547_TORCH_MIN_UA || *flash_ua < RT8547_FLASH_MIN_UA ||
	    *timeout_us < RT8547_TIMEOUT_MIN_US)
		return dev_err_probe(dev, -EINVAL, "LED limits below minimum\n");

	*torch_ua = min(*torch_ua, RT8547_TORCH_MAX_UA);
	*flash_ua = min(*flash_ua, RT8547_FLASH_MAX_UA);
	*timeout_us = min(*timeout_us, RT8547_TIMEOUT_MAX_US);

	return 0;
}

static int rt8547_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_flash_config v4l2_cfg = {};
	struct led_init_data init_data = {};
	u32 torch_ua, flash_ua, timeout_us;
	struct led_flash_setting *s;
	struct led_classdev *led;
	struct rt8547 *rt;
	int ret;

	rt = devm_kzalloc_obj(dev, *rt);
	if (!rt)
		return -ENOMEM;

	rt->dev = dev;
	rt->props.timing = rt8547_timing;
	led = &rt->fled.led_cdev;
	platform_set_drvdata(pdev, rt);

	rt->props.ctrl_gpio = devm_gpiod_get(dev, "flset", GPIOD_OUT_LOW);
	if (IS_ERR(rt->props.ctrl_gpio))
		return dev_err_probe(dev, PTR_ERR(rt->props.ctrl_gpio),
				     "cannot get FLSET GPIO\n");

	rt->flen_gpio = devm_gpiod_get(dev, "flen", GPIOD_OUT_LOW);
	if (IS_ERR(rt->flen_gpio))
		return dev_err_probe(dev, PTR_ERR(rt->flen_gpio),
				     "cannot get FLEN GPIO\n");

	ret = devm_regulator_get_enable_optional(dev, "vin");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "cannot enable VIN supply\n");

	struct fwnode_handle *child __free(fwnode_handle) =
		device_get_next_child_node(dev, NULL);
	if (!child)
		return dev_err_probe(dev, -EINVAL, "no LED node\n");

	ret = rt8547_parse_led(rt, child, &torch_ua, &flash_ua, &timeout_us);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &rt->lock);
	if (ret)
		return ret;

	timer_setup(&rt->strobe_timer, rt8547_strobe_timeout, 0);
	ret = devm_add_action_or_reset(dev, rt8547_disable, rt);
	if (ret)
		return ret;

	s = &rt->fled.brightness;
	s->min = RT8547_FLASH_MIN_UA;
	s->step = RT8547_FLASH_STEP_UA;
	s->max = rounddown(flash_ua - s->min, s->step) + s->min;
	s->val = s->max;

	s = &rt->fled.timeout;
	s->min = RT8547_TIMEOUT_MIN_US;
	s->step = RT8547_TIMEOUT_STEP_US;
	s->max = rounddown(timeout_us - s->min, s->step) + s->min;
	s->val = s->max;

	rt->fled.ops = &rt8547_flash_ops;
	led->max_brightness = torch_ua / RT8547_TORCH_STEP_UA;
	led->brightness_set_blocking = rt8547_brightness_set;
	led->flags |= LED_CORE_SUSPENDRESUME | LED_DEV_CAP_FLASH;

	init_data.fwnode = child;
	ret = devm_led_classdev_flash_register_ext(dev, &rt->fled, &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register LED\n");

	strscpy(v4l2_cfg.dev_name, dev_name(led->dev));
	s = &v4l2_cfg.intensity;
	s->min = RT8547_TORCH_MIN_UA;
	s->step = RT8547_TORCH_STEP_UA;
	s->max = led->max_brightness * RT8547_TORCH_STEP_UA;
	s->val = s->max;

	rt->v4l2_flash = v4l2_flash_init(dev, child, &rt->fled, NULL, &v4l2_cfg);
	if (IS_ERR(rt->v4l2_flash))
		return dev_err_probe(dev, PTR_ERR(rt->v4l2_flash),
				     "cannot register V4L2 flash device\n");

	return devm_add_action_or_reset(dev, rt8547_v4l2_flash_release,
					rt->v4l2_flash);
}

static void rt8547_shutdown(struct platform_device *pdev)
{
	rt8547_disable(platform_get_drvdata(pdev));
}

static const struct of_device_id rt8547_of_match[] = {
	{ .compatible = "richtek,rt8547" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt8547_of_match);

static struct platform_driver rt8547_driver = {
	.driver = {
		.name = "leds-rt8547",
		.of_match_table = rt8547_of_match,
	},
	.probe = rt8547_probe,
	.shutdown = rt8547_shutdown,
};
module_platform_driver(rt8547_driver);

MODULE_IMPORT_NS("EXPRESSWIRE");
MODULE_AUTHOR("Martin Štrobl <recordyletsplays@gmail.com>");
MODULE_DESCRIPTION("Richtek RT8547 flash LED driver");
MODULE_LICENSE("GPL");
