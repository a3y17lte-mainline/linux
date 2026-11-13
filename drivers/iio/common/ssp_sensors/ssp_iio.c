// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2014, Samsung Electronics Co. Ltd. All Rights Reserved.
 */

#include <linux/cleanup.h>
#include <linux/iio/common/ssp_sensors.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include "ssp_iio_sensor.h"

/**
 * ssp_common_buffer_postenable() - generic postenable callback for ssp buffer
 *
 * @indio_dev:		iio device
 *
 * Returns 0 or negative value in case of error
 */
int ssp_common_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ssp_sensor_data *spd = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);

	/* the allocation is made in post because scan size is known in this
	 * moment
	 * */
	spd->buffer = kmalloc(indio_dev->scan_bytes, GFP_KERNEL | GFP_DMA);
	if (!spd->buffer)
		return -ENOMEM;

	return ssp_enable_sensor(data, spd->type,
				 ssp_get_sensor_delay(data, spd->type));
}
EXPORT_SYMBOL_NS(ssp_common_buffer_postenable, "IIO_SSP_SENSORS");

/**
 * ssp_common_buffer_postdisable() - generic postdisable callback for ssp buffer
 *
 * @indio_dev:		iio device
 *
 * Returns 0 or negative value in case of error
 */
int ssp_common_buffer_postdisable(struct iio_dev *indio_dev)
{
	int ret;
	struct ssp_sensor_data *spd = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);

	ret = ssp_disable_sensor(data, spd->type);
	if (ret < 0)
		return ret;

	kfree(spd->buffer);

	return ret;
}
EXPORT_SYMBOL_NS(ssp_common_buffer_postdisable, "IIO_SSP_SENSORS");

/**
 * ssp_common_process_data() - Common process data callback for ssp sensors
 *
 * @indio_dev:		iio device
 * @buf:		source buffer
 * @len:		sensor data length
 * @timestamp:		system timestamp
 *
 * Returns 0 or negative value in case of error
 */
int ssp_common_process_data(struct iio_dev *indio_dev, void *buf,
			    unsigned int len, int64_t timestamp)
{
	struct ssp_sensor_data *spd = iio_priv(indio_dev);

	scoped_guard(mutex, &spd->sample_lock)
		memcpy(spd->sample, buf, min(len, SSP_MAX_SAMPLE_SIZE));
	complete(&spd->sample_ready);

	if (!iio_buffer_enabled(indio_dev) || !spd->buffer ||
	    indio_dev->scan_bytes == 0)
		return 0;

	/*
	 * it always sends full set of samples, remember about available masks
	 */
	memcpy(spd->buffer, buf, len);

	return iio_push_to_buffers_with_timestamp(indio_dev, spd->buffer,
						  timestamp);
}
EXPORT_SYMBOL_NS(ssp_common_process_data, "IIO_SSP_SENSORS");

/**
 * ssp_common_init() - sets up the sensor object of an SSP IIO device
 * @indio_dev:		IIO device
 * @type:		SSP sensor type
 * @process_data:	callback for the sensor's samples
 *
 * Returns 0 or negative value in case of error
 */
int ssp_common_init(struct iio_dev *indio_dev, enum ssp_sensor_type type,
		    int (*process_data)(struct iio_dev *indio_dev, void *buf,
					int64_t timestamp))
{
	struct ssp_sensor_data *spd = iio_priv(indio_dev);

	spd->type = type;
	spd->process_data = process_data;
	init_completion(&spd->sample_ready);

	return devm_mutex_init(indio_dev->dev.parent, &spd->sample_lock);
}
EXPORT_SYMBOL_NS(ssp_common_init, "IIO_SSP_SENSORS");

#define SSP_SAMPLE_TIMEOUT_MS	1000

/* enables the sensor until it delivers a sample */
static int ssp_read_one_sample(struct ssp_data *data,
			       struct ssp_sensor_data *spd)
{
	int ret;

	reinit_completion(&spd->sample_ready);

	ret = ssp_enable_sensor(data, spd->type,
				ssp_get_sensor_delay(data, spd->type));
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&spd->sample_ready,
					 msecs_to_jiffies(SSP_SAMPLE_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	ssp_disable_sensor(data, spd->type);

	return ret;
}

/**
 * ssp_common_read_sample() - reads part of a sample of the sensor
 * @indio_dev:	IIO device
 * @dst:	destination
 * @offset:	offset of the value in the sample
 * @len:	size of the value
 *
 * When the buffer is running, or the driver keeps the sensor running, this
 * returns the value from the last sample. Otherwise the sensor is enabled
 * until it delivers a sample.
 *
 * Returns 0 or negative value in case of error
 */
int ssp_common_read_sample(struct iio_dev *indio_dev, void *dst,
			   unsigned int offset, unsigned int len)
{
	struct ssp_sensor_data *spd = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);
	int ret = 0;

	if (offset + len > SSP_MAX_SAMPLE_SIZE)
		return -EINVAL;

	if (iio_device_claim_direct(indio_dev)) {
		if (!spd->keep_on)
			ret = ssp_read_one_sample(data, spd);

		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
	}

	guard(mutex)(&spd->sample_lock);
	memcpy(dst, spd->sample + offset, len);

	return 0;
}
EXPORT_SYMBOL_NS(ssp_common_read_sample, "IIO_SSP_SENSORS");

/**
 * ssp_common_read_raw() - read_raw for simple SSP sensors
 * @indio_dev:	IIO device
 * @chan:	channel; its address is the byte offset of its value in the
 *		sample, its scan type gives the size and sign
 * @val:	value
 * @val2:	second part of the value
 * @mask:	what to read: the raw or processed value, the sampling
 *		frequency, or the scale of the sensor from the hub info
 *
 * Returns the IIO value type or a negative value in case of error
 */
int ssp_common_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan, int *val, int *val2,
			long mask)
{
	struct ssp_sensor_data *spd = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);
	unsigned int bytes = chan->scan_type.storagebits / 8;
	u8 raw[4];
	u32 v;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		if (bytes > sizeof(raw))
			return -EINVAL;

		ret = ssp_common_read_sample(indio_dev, raw, chan->address,
					     bytes);
		if (ret)
			return ret;

		switch (bytes) {
		case 1:
			v = raw[0];
			break;
		case 2:
			v = get_unaligned_le16(raw);
			break;
		default:
			v = get_unaligned_le32(raw);
			break;
		}

		*val = chan->scan_type.sign == 's' ?
		       sign_extend32(v, chan->scan_type.realbits - 1) : v;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ssp_convert_to_freq(ssp_get_sensor_delay(data, spd->type), val,
				    val2);
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		return ssp_get_sensor_scale(data, spd->type, val, val2);
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_NS(ssp_common_read_raw, "IIO_SSP_SENSORS");

/**
 * ssp_common_write_raw() - write_raw for simple SSP sensors
 * @indio_dev:	IIO device
 * @chan:	channel
 * @val:	value
 * @val2:	second part of the value
 * @mask:	what to write, only the sampling frequency
 *
 * Returns 0 or negative value in case of error
 */
int ssp_common_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan, int val, int val2,
			 long mask)
{
	struct ssp_sensor_data *spd = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	return ssp_change_delay(data, spd->type, ssp_convert_to_time(val, val2));
}
EXPORT_SYMBOL_NS(ssp_common_write_raw, "IIO_SSP_SENSORS");

MODULE_AUTHOR("Karol Wrona <k.wrona@samsung.com>");
MODULE_DESCRIPTION("Samsung sensorhub commons");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_SSP_SENSORS");
