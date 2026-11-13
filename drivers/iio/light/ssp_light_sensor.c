// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ambient light and colour sensor behind the Samsung sensorhub
 */

#include <linux/iio/buffer.h>
#include <linux/iio/common/ssp_sensors.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "../common/ssp_sensors/ssp_iio_sensor.h"

static const struct iio_chan_spec ssp_light_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_COLORTEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 4,
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_RED,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 8,
		.scan_index = 2,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_GREEN,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 10,
		.scan_index = 3,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_BLUE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 12,
		.scan_index = 4,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_CLEAR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 14,
		.scan_index = 5,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(6),
};

static const unsigned long ssp_light_scan_masks[] = { 0x3f, 0 };

static int ssp_light_process_data(struct iio_dev *indio_dev, void *buf,
				  int64_t timestamp)
{
	/* lux and colour temperature are computed by the hub */
	return ssp_common_process_data(indio_dev, buf, SSP_LIGHT_SIZE, timestamp);
}

static const struct iio_info ssp_light_iio_info = {
	.read_raw = ssp_common_read_raw,
	.write_raw = ssp_common_write_raw,
};

static const struct iio_buffer_setup_ops ssp_light_buffer_ops = {
	.postenable = ssp_common_buffer_postenable,
	.postdisable = ssp_common_buffer_postdisable,
};

static int ssp_light_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev,
					  sizeof(struct ssp_sensor_data));
	if (!indio_dev)
		return -ENOMEM;

	ret = ssp_common_init(indio_dev, SSP_LIGHT_SENSOR,
			      ssp_light_process_data);
	if (ret)
		return ret;

	indio_dev->name = "ssp-light";
	indio_dev->info = &ssp_light_iio_info;
	indio_dev->channels = ssp_light_channels;
	indio_dev->num_channels = ARRAY_SIZE(ssp_light_channels);
	indio_dev->available_scan_masks = ssp_light_scan_masks;

	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  &ssp_light_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	/* ssp registering should be done after all iio setup */
	return ssp_register_consumer(indio_dev, SSP_LIGHT_SENSOR);
}

static const struct platform_device_id ssp_light_id[] = {
	{ "ssp-light" },
	{ }
};
MODULE_DEVICE_TABLE(platform, ssp_light_id);

static struct platform_driver ssp_light_driver = {
	.driver = {
		.name = "ssp-light",
	},
	.probe = ssp_light_probe,
	.id_table = ssp_light_id,
};
module_platform_driver(ssp_light_driver);

MODULE_AUTHOR("Martin Štrobl <recordyletsplays@gmail.com>");
MODULE_DESCRIPTION("Samsung sensorhub light sensor driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_SSP_SENSORS");
