// SPDX-License-Identifier: GPL-2.0-only
/*
 * Proximity sensor behind the Samsung sensorhub
 */

#include <linux/iio/buffer.h>
#include <linux/iio/common/ssp_sensors.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "../common/ssp_sensors/ssp_iio_sensor.h"

/* hub decision not known yet: the next sample is reported as an event */
#define SSP_PROX_STATE_UNKNOWN	0xff

struct ssp_prox_data {
	struct ssp_sensor_data spd;	/* must be first, see ssp_common_init() */
	u32 near_level;
	u32 far_level;
	u8 state;
};

static struct ssp_data *ssp_prox_hub(struct iio_dev *indio_dev)
{
	return dev_get_drvdata(indio_dev->dev.parent->parent);
}

static ssize_t ssp_prox_read_near_level(struct iio_dev *indio_dev,
					uintptr_t priv,
					const struct iio_chan_spec *chan,
					char *buf)
{
	struct ssp_prox_data *prox = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", prox->near_level);
}

static const struct iio_chan_spec_ext_info ssp_prox_ext_info[] = {
	{
		.name = "nearlevel",
		.shared = IIO_SEPARATE,
		.read = ssp_prox_read_near_level,
	},
	{ }
};

static const struct iio_event_spec ssp_prox_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

#define SSP_PROX_CHANNEL(_events, _num_events, _ext_info) {		\
	.type = IIO_PROXIMITY,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.address = 0,							\
	.scan_index = 0,						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 8,						\
		.storagebits = 8,					\
		.endianness = IIO_LE,					\
	},								\
	.event_spec = _events,						\
	.num_event_specs = _num_events,					\
	.ext_info = _ext_info,						\
}

/* with thresholds from the board: near level and near/far events */
static const struct iio_chan_spec ssp_prox_channels[] = {
	SSP_PROX_CHANNEL(ssp_prox_events, ARRAY_SIZE(ssp_prox_events),
			 ssp_prox_ext_info),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec ssp_prox_channels_raw[] = {
	SSP_PROX_CHANNEL(NULL, 0, NULL),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const unsigned long ssp_prox_scan_masks[] = { 0x1, 0 };

static int ssp_prox_process_data(struct iio_dev *indio_dev, void *buf,
				 int64_t timestamp)
{
	struct ssp_prox_data *prox = iio_priv(indio_dev);
	u8 near = *(u8 *)buf;
	enum iio_event_direction dir;

	/*
	 * The sample is the hub's near/far decision, made with the board's
	 * thresholds, followed by the raw proximity count.
	 */
	if (READ_ONCE(prox->spd.keep_on) && near != prox->state) {
		dir = near ? IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 0,
						    IIO_EV_TYPE_THRESH, dir),
			       timestamp);
	}
	prox->state = near;

	return ssp_common_process_data(indio_dev, (u8 *)buf + 1, 1, timestamp);
}

static int ssp_prox_read_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int *val, int *val2)
{
	struct ssp_prox_data *prox = iio_priv(indio_dev);

	switch (dir) {
	case IIO_EV_DIR_RISING:
		*val = prox->near_level;
		return IIO_VAL_INT;
	case IIO_EV_DIR_FALLING:
		*val = prox->far_level;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ssp_prox_read_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir)
{
	struct ssp_prox_data *prox = iio_priv(indio_dev);

	return READ_ONCE(prox->spd.keep_on);
}

/* called with the IIO device mode claimed */
static int ssp_prox_set_events(struct iio_dev *indio_dev, bool state,
			       bool sensor_on)
{
	struct ssp_prox_data *prox = iio_priv(indio_dev);
	struct ssp_data *data = ssp_prox_hub(indio_dev);
	int ret = 0;

	if (state == prox->spd.keep_on)
		return 0;

	if (state) {
		prox->state = SSP_PROX_STATE_UNKNOWN;
		if (!sensor_on)
			ret = ssp_enable_sensor(data, prox->spd.type,
						ssp_get_sensor_delay(data,
								     prox->spd.type));
	} else if (!sensor_on) {
		ret = ssp_disable_sensor(data, prox->spd.type);
	}
	if (ret)
		return ret;

	WRITE_ONCE(prox->spd.keep_on, state);

	return 0;
}

static int ssp_prox_write_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       bool state)
{
	int ret;

	/* in buffer mode the buffer keeps the sensor running */
	if (iio_device_claim_direct(indio_dev)) {
		ret = ssp_prox_set_events(indio_dev, state, false);
		iio_device_release_direct(indio_dev);
		return ret;
	}

	if (!iio_device_try_claim_buffer_mode(indio_dev))
		return -EBUSY;
	ret = ssp_prox_set_events(indio_dev, state, true);
	iio_device_release_buffer_mode(indio_dev);

	return ret;
}

static const struct iio_info ssp_prox_iio_info = {
	.read_raw = ssp_common_read_raw,
	.write_raw = ssp_common_write_raw,
	.read_event_value = ssp_prox_read_event_value,
	.read_event_config = ssp_prox_read_event_config,
	.write_event_config = ssp_prox_write_event_config,
};

/* while events are on, the sensor is already running */
static int ssp_prox_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ssp_prox_data *prox = iio_priv(indio_dev);

	if (prox->spd.keep_on)
		return 0;

	return ssp_common_buffer_postenable(indio_dev);
}

static int ssp_prox_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ssp_prox_data *prox = iio_priv(indio_dev);

	if (prox->spd.keep_on)
		return 0;

	return ssp_common_buffer_postdisable(indio_dev);
}

static const struct iio_buffer_setup_ops ssp_prox_buffer_ops = {
	.postenable = ssp_prox_buffer_postenable,
	.postdisable = ssp_prox_buffer_postdisable,
};

static int ssp_prox_probe(struct platform_device *pdev)
{
	struct ssp_data *data = dev_get_drvdata(pdev->dev.parent);
	struct ssp_prox_data *prox;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*prox));
	if (!indio_dev)
		return -ENOMEM;

	ret = ssp_common_init(indio_dev, SSP_PROXIMITY_SENSOR,
			      ssp_prox_process_data);
	if (ret)
		return ret;

	prox = iio_priv(indio_dev);
	prox->state = SSP_PROX_STATE_UNKNOWN;

	indio_dev->name = "ssp-proximity";
	indio_dev->info = &ssp_prox_iio_info;
	if (ssp_get_proximity_thresholds(data, &prox->near_level,
					 &prox->far_level)) {
		indio_dev->channels = ssp_prox_channels_raw;
		indio_dev->num_channels = ARRAY_SIZE(ssp_prox_channels_raw);
	} else {
		indio_dev->channels = ssp_prox_channels;
		indio_dev->num_channels = ARRAY_SIZE(ssp_prox_channels);
	}
	indio_dev->available_scan_masks = ssp_prox_scan_masks;

	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  &ssp_prox_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	/* ssp registering should be done after all iio setup */
	return ssp_register_consumer(indio_dev, SSP_PROXIMITY_SENSOR);
}

static const struct platform_device_id ssp_prox_id[] = {
	{ "ssp-proximity" },
	{ }
};
MODULE_DEVICE_TABLE(platform, ssp_prox_id);

static struct platform_driver ssp_prox_driver = {
	.driver = {
		.name = "ssp-proximity",
	},
	.probe = ssp_prox_probe,
	.id_table = ssp_prox_id,
};
module_platform_driver(ssp_prox_driver);

MODULE_AUTHOR("Martin Štrobl <recordyletsplays@gmail.com>");
MODULE_DESCRIPTION("Samsung sensorhub proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_SSP_SENSORS");
