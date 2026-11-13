// SPDX-License-Identifier: GPL-2.0-only
/*
 * Barometer behind the Samsung sensorhub
 */

#include <linux/iio/buffer.h>
#include <linux/iio/common/ssp_sensors.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "../common/ssp_sensors/ssp_iio_sensor.h"

static const struct iio_chan_spec ssp_pressure_channels[] = {
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 4,
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const unsigned long ssp_pressure_scan_masks[] = { 0x3, 0 };

static int ssp_pressure_process_data(struct iio_dev *indio_dev, void *buf,
				     int64_t timestamp)
{
	return ssp_common_process_data(indio_dev, buf, SSP_PRESSURE_SIZE,
				       timestamp);
}

/*
 * The hub passes on the LPS25H temperature register:
 * T = 42.5 degrees C + raw / 480.
 */
#define SSP_PRESSURE_TEMP_OFFSET	20400	/* 42.5 * 480 */

static int ssp_pressure_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int *val,
				 int *val2, long mask)
{
	if (chan->type == IIO_TEMP) {
		switch (mask) {
		case IIO_CHAN_INFO_SCALE:
			/* milli degrees C */
			*val = 1000;
			*val2 = 480;
			return IIO_VAL_FRACTIONAL;
		case IIO_CHAN_INFO_OFFSET:
			*val = SSP_PRESSURE_TEMP_OFFSET;
			return IIO_VAL_INT;
		}
	}

	return ssp_common_read_raw(indio_dev, chan, val, val2, mask);
}

static const struct iio_info ssp_pressure_iio_info = {
	.read_raw = ssp_pressure_read_raw,
	.write_raw = ssp_common_write_raw,
};

static const struct iio_buffer_setup_ops ssp_pressure_buffer_ops = {
	.postenable = ssp_common_buffer_postenable,
	.postdisable = ssp_common_buffer_postdisable,
};

static int ssp_pressure_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev,
					  sizeof(struct ssp_sensor_data));
	if (!indio_dev)
		return -ENOMEM;

	ret = ssp_common_init(indio_dev, SSP_PRESSURE_SENSOR,
			      ssp_pressure_process_data);
	if (ret)
		return ret;

	indio_dev->name = "ssp-pressure";
	indio_dev->info = &ssp_pressure_iio_info;
	indio_dev->channels = ssp_pressure_channels;
	indio_dev->num_channels = ARRAY_SIZE(ssp_pressure_channels);
	indio_dev->available_scan_masks = ssp_pressure_scan_masks;

	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  &ssp_pressure_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	/* ssp registering should be done after all iio setup */
	return ssp_register_consumer(indio_dev, SSP_PRESSURE_SENSOR);
}

static const struct platform_device_id ssp_pressure_id[] = {
	{ "ssp-pressure" },
	{ }
};
MODULE_DEVICE_TABLE(platform, ssp_pressure_id);

static struct platform_driver ssp_pressure_driver = {
	.driver = {
		.name = "ssp-pressure",
	},
	.probe = ssp_pressure_probe,
	.id_table = ssp_pressure_id,
};
module_platform_driver(ssp_pressure_driver);

MODULE_AUTHOR("Martin Štrobl <recordyletsplays@gmail.com>");
MODULE_DESCRIPTION("Samsung sensorhub barometer driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_SSP_SENSORS");
