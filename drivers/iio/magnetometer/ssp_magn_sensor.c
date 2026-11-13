// SPDX-License-Identifier: GPL-2.0-only
/*
 * Magnetometer behind the Samsung sensorhub
 */

#include <linux/iio/buffer.h>
#include <linux/iio/common/ssp_sensors.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "../common/ssp_sensors/ssp_iio_sensor.h"

static const struct iio_chan_spec ssp_magn_channels[] = {
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 2,
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.address = 4,
		.scan_index = 2,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const unsigned long ssp_magn_scan_masks[] = { 0x7, 0 };

static int ssp_magn_process_data(struct iio_dev *indio_dev, void *buf,
				 int64_t timestamp)
{
	/* the hub appends a calibration accuracy byte, not exposed */
	return ssp_common_process_data(indio_dev, buf, SSP_MAGNETOMETER_SIZE,
				       timestamp);
}

static const struct iio_info ssp_magn_iio_info = {
	.read_raw = ssp_common_read_raw,
	.write_raw = ssp_common_write_raw,
};

static const struct iio_buffer_setup_ops ssp_magn_buffer_ops = {
	.postenable = ssp_common_buffer_postenable,
	.postdisable = ssp_common_buffer_postdisable,
};

static int ssp_magn_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev,
					  sizeof(struct ssp_sensor_data));
	if (!indio_dev)
		return -ENOMEM;

	ret = ssp_common_init(indio_dev, SSP_GEOMAGNETIC_SENSOR,
			      ssp_magn_process_data);
	if (ret)
		return ret;

	indio_dev->name = "ssp-magnetometer";
	indio_dev->info = &ssp_magn_iio_info;
	indio_dev->channels = ssp_magn_channels;
	indio_dev->num_channels = ARRAY_SIZE(ssp_magn_channels);
	indio_dev->available_scan_masks = ssp_magn_scan_masks;

	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  &ssp_magn_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	/* ssp registering should be done after all iio setup */
	return ssp_register_consumer(indio_dev, SSP_GEOMAGNETIC_SENSOR);
}

static const struct platform_device_id ssp_magn_id[] = {
	{ "ssp-magnetometer" },
	{ }
};
MODULE_DEVICE_TABLE(platform, ssp_magn_id);

static struct platform_driver ssp_magn_driver = {
	.driver = {
		.name = "ssp-magnetometer",
	},
	.probe = ssp_magn_probe,
	.id_table = ssp_magn_id,
};
module_platform_driver(ssp_magn_driver);

MODULE_AUTHOR("Martin Štrobl <recordyletsplays@gmail.com>");
MODULE_DESCRIPTION("Samsung sensorhub magnetometer driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_SSP_SENSORS");
