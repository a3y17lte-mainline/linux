// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2014, Samsung Electronics Co. Ltd. All Rights Reserved.
 */

#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/property.h>

#include "ssp.h"

#define SSP_WDT_TIME			10000
#define SSP_LIMIT_RESET_CNT		20
#define SSP_LIMIT_TIMEOUT_CNT		3

/* It is possible that it is max clk rate for version 1.0 of bootcode */
#define SSP_BOOT_SPI_HZ	400000

/*
 * These fields can look enigmatic but this structure is used mainly to flat
 * some values and depends on command type.
 */
struct ssp_instruction {
	__le32 a;
	__le32 b;
	u8 c;
} __packed;

static const u8 ssp_magnitude_table[] = {110, 85, 171, 71, 203, 195, 0, 67,
	208, 56, 175, 244, 206, 213, 0, 92, 250, 0, 55, 48, 189, 252, 171,
	243, 13, 45, 250};

/* Sensor types of the Galaxy A3 (2017) firmware: the Android ones */
static const u8 ssp_a3y17lte_wire_types[SSP_SENSOR_MAX] = {
	[SSP_ACCELEROMETER_SENSOR] = 1,
	[SSP_GYROSCOPE_SENSOR] = 4,
	[SSP_GEOMAGNETIC_UNCALIB_SENSOR] = 14,
	[SSP_GEOMAGNETIC_SENSOR] = 2,
	[SSP_PRESSURE_SENSOR] = 6,
	[SSP_PROXIMITY_SENSOR] = 8,
	[SSP_LIGHT_SENSOR] = 5,
	[SSP_PROXIMITY_RAW] = 29,
	[SSP_STEP_DETECTOR] = 18,
	[SSP_SIG_MOTION_SENSOR] = 17,
	[SSP_GYRO_UNCALIB_SENSOR] = 16,
	[SSP_GAME_ROTATION_VECTOR] = 15,
	[SSP_ROTATION_VECTOR] = 11,
	[SSP_STEP_COUNTER] = 19,
};

/*
 * +-8 g with 4096 LSB/g, +-1000 dps with 28.57 LSB/dps, the magnetic field in
 * 0.06 uT (the vendor sensor HAL's resolution), the pressure in 1/4096 hPa
 * (the LPS25H's own unit)
 */
static const int ssp_a3y17lte_scales[SSP_SENSOR_MAX][2] = {
	[SSP_ACCELEROMETER_SENSOR] = { 0, 2394202 },
	[SSP_GYROSCOPE_SENSOR] = { 0, 610865 },
	[SSP_GEOMAGNETIC_SENSOR] = { 0, 600000 },
	[SSP_PRESSURE_SENSOR] = { 0, 24414 },
};

static const u8 ssp_a3y17lte_sample_sizes[SSP_SENSOR_MAX] = {
	[SSP_ACCELEROMETER_SENSOR] = SSP_ACCELEROMETER_SIZE,
	[SSP_GYROSCOPE_SENSOR] = SSP_GYROSCOPE_SIZE,
	[SSP_GEOMAGNETIC_UNCALIB_SENSOR] = 12,
	[SSP_GEOMAGNETIC_SENSOR] = SSP_MAGNETOMETER_SIZE,
	[SSP_PRESSURE_SENSOR] = SSP_PRESSURE_SIZE,
	[SSP_PROXIMITY_SENSOR] = SSP_PROXIMITY_SIZE,
	[SSP_LIGHT_SENSOR] = SSP_LIGHT_SIZE,
	[SSP_PROXIMITY_RAW] = 1,
	[SSP_STEP_DETECTOR] = 1,
	[SSP_SIG_MOTION_SENSOR] = 1,
	[SSP_GYRO_UNCALIB_SENSOR] = 12,
	[SSP_STEP_COUNTER] = 4,
};

static const struct mfd_cell ssp_a3y17lte_cells[] = {
	{ .name = "ssp-accelerometer" },
	{ .name = "ssp-gyroscope" },
	{ .name = "ssp-magnetometer" },
	{ .name = "ssp-pressure" },
	{ .name = "ssp-light" },
	{ .name = "ssp-proximity" },
};

static int ssp_a3y17lte_pre_scan(struct ssp_data *data)
{
	/* the gyroscope is in the same package as the accelerometer */
	u8 formation[] = {
		data->accel_position, data->accel_position, data->mag_position,
	};

	return ssp_write_command(data, SSP_MSG2SSP_AP_SENSOR_FORMATION,
				 formation, sizeof(formation));
}

static int ssp_a3y17lte_post_scan(struct ssp_data *data)
{
	static const u8 mag_matrix[SSP_MAG_MATRIX_SIZE];
	__le32 coef[SSP_LIGHT_COEF_COUNT];
	int ret, i;

	if (data->available_sensors & BIT(SSP_GEOMAGNETIC_SENSOR)) {
		ret = ssp_write_command(data,
					SSP_MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX,
					mag_matrix, sizeof(mag_matrix));
		if (ret < 0)
			return ret;
	}

	if (data->has_light_coef &&
	    data->available_sensors & BIT(SSP_LIGHT_SENSOR)) {
		for (i = 0; i < SSP_LIGHT_COEF_COUNT; i++)
			coef[i] = cpu_to_le32(data->light_coef[i]);

		ret = ssp_write_command(data, SSP_MSG2SSP_AP_SET_LIGHT_COEF,
					coef, sizeof(coef));
		if (ret < 0)
			return ret;
	}

	if (data->has_prox_thresh &&
	    data->available_sensors & BIT(SSP_PROXIMITY_SENSOR)) {
		u8 thresh[SSP_PROX_THRESH_COUNT];

		for (i = 0; i < SSP_PROX_THRESH_COUNT; i++)
			thresh[i] = data->prox_thresh[i];

		ret = ssp_write_command(data,
					SSP_MSG2SSP_AP_SENSOR_PROXTHRESHOLD,
					thresh, sizeof(thresh));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct ssp_sensorhub_info ssp_a3y17lte_info = {
	.wire_types = ssp_a3y17lte_wire_types,
	.sample_sizes = ssp_a3y17lte_sample_sizes,
	.cells = ssp_a3y17lte_cells,
	.num_cells = ARRAY_SIZE(ssp_a3y17lte_cells),
	.scan_size = 8,
	.max_delay = 200,
	.boot_time = 500,
	.batched = true,
	.on_change = BIT(SSP_LIGHT_SENSOR) | BIT(SSP_PROXIMITY_SENSOR),
	.scales = ssp_a3y17lte_scales,
	.pre_scan = ssp_a3y17lte_pre_scan,
	.post_scan = ssp_a3y17lte_post_scan,
};

static const struct ssp_sensorhub_info ssp_rinato_info = {
	.fw_name = "ssp_B2.fw",
	.fw_crashed_name = "ssp_crashed.fw",
	.fw_rev = 14052300,
	.mag_table = ssp_magnitude_table,
	.mag_length = ARRAY_SIZE(ssp_magnitude_table),
	.scan_size = 4,
	.inst_pad = 1,
	.dump_mode = true,
	.pre_scan = ssp_set_magnetic_matrix,
};

static const struct ssp_sensorhub_info ssp_thermostat_info = {
	.fw_name = "thermostat_B2.fw",
	.fw_crashed_name = "ssp_crashed.fw",
	.fw_rev = 14080600,
	.mag_table = ssp_magnitude_table,
	.mag_length = ARRAY_SIZE(ssp_magnitude_table),
	.scan_size = 4,
	.inst_pad = 1,
	.dump_mode = true,
	.pre_scan = ssp_set_magnetic_matrix,
};

static const struct mfd_cell sensorhub_sensor_devs[] = {
	{
		.name = "ssp-accelerometer",
	},
	{
		.name = "ssp-gyroscope",
	},
};

static void ssp_toggle_mcu_reset_gpio(struct ssp_data *data)
{
	gpiod_set_value(data->mcu_reset_gpiod, 0);
	usleep_range(1000, 1200);
	gpiod_set_value(data->mcu_reset_gpiod, 1);
	msleep(50);
}

static int ssp_send_delay(struct ssp_data *data, u8 inst,
			  enum ssp_sensor_type type, u32 delay)
{
	struct ssp_instruction to_send;

	if (data->sensorhub_info->max_delay)
		delay = min(delay, data->sensorhub_info->max_delay);

	to_send.a = cpu_to_le32(delay);
	to_send.b = cpu_to_le32(data->batch_latency_buf[type]);
	to_send.c = data->batch_opt_buf[type];

	return ssp_send_instruction(data, inst, type, (u8 *)&to_send,
				    sizeof(to_send));
}

static void ssp_sync_available_sensors(struct ssp_data *data)
{
	int i, ret;

	for (i = 0; i < SSP_SENSOR_MAX; ++i) {
		if (data->sensor_enable & BIT(i)) {
			data->last_ts[i] = 0;
			ret = ssp_send_delay(data,
					     SSP_MSG2SSP_INST_BYPASS_SENSOR_ADD,
					     i, data->delay_buf[i]);
			if (ret < 0) {
				dev_err(&data->spi->dev,
					"Sync sensor nr: %d fail\n", i);
				continue;
			}
		}
	}

	if (!data->sensorhub_info->dump_mode)
		return;

	ret = ssp_command(data, SSP_MSG2SSP_AP_MCU_SET_DUMPMODE,
			  data->mcu_dump_mode);
	if (ret < 0)
		dev_err(&data->spi->dev,
			"SSP_MSG2SSP_AP_MCU_SET_DUMPMODE failed\n");
}

static void ssp_enable_mcu(struct ssp_data *data, bool enable)
{
	dev_info(&data->spi->dev, "current shutdown = %d, old = %d\n", enable,
		 data->shut_down);

	if (enable && data->shut_down) {
		data->shut_down = false;
		enable_irq(data->spi->irq);
		enable_irq_wake(data->spi->irq);
	} else if (!enable && !data->shut_down) {
		data->shut_down = true;
		disable_irq(data->spi->irq);
		disable_irq_wake(data->spi->irq);
	} else {
		dev_warn(&data->spi->dev, "current shutdown = %d, old = %d\n",
			 enable, data->shut_down);
	}
}

/*
 * This function is the first one which communicates with the mcu so it is
 * possible that the first attempt will fail
 */
static int ssp_check_fwbl(struct ssp_data *data)
{
	int retries = 0;

	while (retries++ < 5) {
		data->cur_firm_rev = ssp_get_firmware_rev(data);
		if (data->cur_firm_rev == SSP_INVALID_REVISION ||
		    data->cur_firm_rev == SSP_INVALID_REVISION2) {
			dev_warn(&data->spi->dev,
				 "Invalid revision, trying %d time\n", retries);
		} else {
			break;
		}
	}

	if (data->cur_firm_rev == SSP_INVALID_REVISION ||
	    data->cur_firm_rev == SSP_INVALID_REVISION2) {
		dev_err(&data->spi->dev, "SSP_INVALID_REVISION\n");
		return SSP_FW_DL_STATE_NEED_TO_SCHEDULE;
	}

	/* any valid revision will do */
	if (!data->sensorhub_info->fw_rev)
		return SSP_FW_DL_STATE_NONE;

	dev_info(&data->spi->dev,
		 "MCU Firm Rev : Old = %8u, New = %8u\n",
		 data->cur_firm_rev,
		 data->sensorhub_info->fw_rev);

	if (data->cur_firm_rev != data->sensorhub_info->fw_rev)
		return SSP_FW_DL_STATE_NEED_TO_SCHEDULE;

	return SSP_FW_DL_STATE_NONE;
}

static void ssp_reset_mcu(struct ssp_data *data)
{
	ssp_enable_mcu(data, false);
	ssp_clean_pending_list(data);
	ssp_toggle_mcu_reset_gpio(data);
	ssp_enable_mcu(data, true);
}

static void ssp_wdt_work_func(struct work_struct *work)
{
	struct ssp_data *data = container_of(work, struct ssp_data, work_wdt);

	dev_err(&data->spi->dev, "%s - Sensor state: 0x%x, RC: %u, CC: %u\n",
		__func__, data->available_sensors, data->reset_cnt,
		data->com_fail_cnt);

	ssp_reset_mcu(data);
	data->com_fail_cnt = 0;
	data->timeout_cnt = 0;
}

static void ssp_wdt_timer_func(struct timer_list *t)
{
	struct ssp_data *data = timer_container_of(data, t, wdt_timer);

	switch (data->fw_dl_state) {
	case SSP_FW_DL_STATE_FAIL:
	case SSP_FW_DL_STATE_DOWNLOADING:
	case SSP_FW_DL_STATE_SYNC:
		goto _mod;
	}

	if (data->timeout_cnt > SSP_LIMIT_TIMEOUT_CNT ||
	    data->com_fail_cnt > SSP_LIMIT_RESET_CNT)
		queue_work(system_power_efficient_wq, &data->work_wdt);
_mod:
	mod_timer(&data->wdt_timer, jiffies + msecs_to_jiffies(SSP_WDT_TIME));
}

static void ssp_enable_wdt_timer(struct ssp_data *data)
{
	mod_timer(&data->wdt_timer, jiffies + msecs_to_jiffies(SSP_WDT_TIME));
}

static void ssp_disable_wdt_timer(struct ssp_data *data)
{
	timer_delete_sync(&data->wdt_timer);
	cancel_work_sync(&data->work_wdt);
}

/**
 * ssp_get_sensor_delay() - gets sensor data acquisition period
 * @data:	sensorhub structure
 * @type:	SSP sensor type
 *
 * Returns acquisition period in ms
 */
u32 ssp_get_sensor_delay(struct ssp_data *data, enum ssp_sensor_type type)
{
	return data->delay_buf[type];
}
EXPORT_SYMBOL_NS(ssp_get_sensor_delay, "IIO_SSP_SENSORS");

/**
 * ssp_get_sensor_scale() - gets the scale of the sensor's raw values
 * @data:	sensorhub structure
 * @type:	SSP sensor type
 * @val:	integer part of the scale
 * @val2:	nano part of the scale
 *
 * Returns IIO_VAL_INT_PLUS_NANO, or -EINVAL if the scale is not known
 */
int ssp_get_sensor_scale(struct ssp_data *data, enum ssp_sensor_type type,
			 int *val, int *val2)
{
	const int (*scales)[2] = data->sensorhub_info->scales;

	if (!scales || (!scales[type][0] && !scales[type][1]))
		return -EINVAL;

	*val = scales[type][0];
	*val2 = scales[type][1];

	return IIO_VAL_INT_PLUS_NANO;
}
EXPORT_SYMBOL_NS(ssp_get_sensor_scale, "IIO_SSP_SENSORS");

/**
 * ssp_get_proximity_thresholds() - gets the hub's proximity thresholds
 * @data:	sensorhub structure
 * @high:	threshold for the hub's near decision, in raw counts
 * @low:	threshold for the hub's far decision, in raw counts
 *
 * Returns 0, or -ENODATA if the board describes no thresholds
 */
int ssp_get_proximity_thresholds(struct ssp_data *data, u32 *high, u32 *low)
{
	if (!data->has_prox_thresh)
		return -ENODATA;

	*high = data->prox_thresh[0];
	*low = data->prox_thresh[1];

	return 0;
}
EXPORT_SYMBOL_NS(ssp_get_proximity_thresholds, "IIO_SSP_SENSORS");

/**
 * ssp_enable_sensor() - enables data acquisition for sensor
 * @data:	sensorhub structure
 * @type:	SSP sensor type
 * @delay:	delay in ms
 *
 * Returns 0 or negative value in case of error
 */
int ssp_enable_sensor(struct ssp_data *data, enum ssp_sensor_type type,
		      u32 delay)
{
	int ret;

	switch (data->check_status[type]) {
	case SSP_INITIALIZATION_STATE:
		/* do calibration step, now just enable */
	case SSP_ADD_SENSOR_STATE:
		data->last_ts[type] = 0;
		ret = ssp_send_delay(data, SSP_MSG2SSP_INST_BYPASS_SENSOR_ADD,
				     type, delay);
		if (ret < 0) {
			dev_err(&data->spi->dev, "Enabling sensor failed\n");
			data->check_status[type] = SSP_NO_SENSOR_STATE;
			goto derror;
		}

		data->sensor_enable |= BIT(type);
		data->check_status[type] = SSP_RUNNING_SENSOR_STATE;
		break;
	case SSP_RUNNING_SENSOR_STATE:
		ret = ssp_send_delay(data, SSP_MSG2SSP_INST_CHANGE_DELAY, type,
				     delay);
		if (ret < 0) {
			dev_err(&data->spi->dev,
				"Changing sensor delay failed\n");
			goto derror;
		}
		break;
	default:
		data->check_status[type] = SSP_ADD_SENSOR_STATE;
		break;
	}

	data->delay_buf[type] = delay;

	if (atomic_inc_return(&data->enable_refcount) == 1)
		ssp_enable_wdt_timer(data);

	return 0;

derror:
	return ret;
}
EXPORT_SYMBOL_NS(ssp_enable_sensor, "IIO_SSP_SENSORS");

/**
 * ssp_change_delay() - changes data acquisition for sensor
 * @data:	sensorhub structure
 * @type:	SSP sensor type
 * @delay:	delay in ms
 *
 * Returns 0 or negative value in case of error
 */
int ssp_change_delay(struct ssp_data *data, enum ssp_sensor_type type,
		     u32 delay)
{
	int ret;

	/* a stopped sensor gets the delay when it is enabled */
	if (!(data->sensor_enable & BIT(type))) {
		data->delay_buf[type] = delay;
		return 0;
	}

	ret = ssp_send_delay(data, SSP_MSG2SSP_INST_CHANGE_DELAY, type, delay);
	if (ret < 0) {
		dev_err(&data->spi->dev, "Changing sensor delay failed\n");
		return ret;
	}

	data->delay_buf[type] = delay;

	return 0;
}
EXPORT_SYMBOL_NS(ssp_change_delay, "IIO_SSP_SENSORS");

/**
 * ssp_disable_sensor() - disables sensor
 *
 * @data:	sensorhub structure
 * @type:	SSP sensor type
 *
 * Returns 0 or negative value in case of error
 */
int ssp_disable_sensor(struct ssp_data *data, enum ssp_sensor_type type)
{
	int ret;
	__le32 command;

	if (data->sensor_enable & BIT(type)) {
		command = cpu_to_le32(data->delay_buf[type]);

		ret = ssp_send_instruction(data,
					   SSP_MSG2SSP_INST_BYPASS_SENSOR_RM,
					   type, (u8 *)&command,
					   sizeof(command));
		if (ret < 0) {
			dev_err(&data->spi->dev, "Remove sensor fail\n");
			return ret;
		}

		data->sensor_enable &= ~BIT(type);
	}

	data->check_status[type] = SSP_ADD_SENSOR_STATE;

	if (atomic_dec_and_test(&data->enable_refcount))
		ssp_disable_wdt_timer(data);

	return 0;
}
EXPORT_SYMBOL_NS(ssp_disable_sensor, "IIO_SSP_SENSORS");

static irqreturn_t ssp_irq_thread_fn(int irq, void *dev_id)
{
	struct ssp_data *data = dev_id;

	/*
	 * This wrapper is done to preserve error path for ssp_irq_msg, also
	 * it is defined in different file.
	 */
	ssp_irq_msg(data);

	return IRQ_HANDLED;
}

static int ssp_initialize_mcu(struct ssp_data *data)
{
	const struct ssp_sensorhub_info *info = data->sensorhub_info;
	int ret;

	ssp_clean_pending_list(data);

	ret = ssp_get_chipid(data);
	if (ret != SSP_DEVICE_ID) {
		dev_err(&data->spi->dev, "%s - MCU %s ret = %d\n", __func__,
			ret < 0 ? "is not working" : "identification failed",
			ret);
		return ret < 0 ? ret : -ENODEV;
	}

	dev_info(&data->spi->dev, "MCU device ID = %d\n", ret);

	/*
	 * needs clarification, for now do not want to export all transfer
	 * methods to sensors' drivers
	 */
	if (info->pre_scan) {
		ret = info->pre_scan(data);
		if (ret < 0) {
			dev_err(&data->spi->dev, "%s - pre-scan setup failed\n",
				__func__);
			return ret;
		}
	}

	data->available_sensors = ssp_get_sensor_scanning_info(data);
	if (data->available_sensors == 0) {
		dev_err(&data->spi->dev,
			"%s - ssp_get_sensor_scanning_info failed\n", __func__);
		return -EIO;
	}

	if (info->post_scan) {
		ret = info->post_scan(data);
		if (ret < 0) {
			dev_err(&data->spi->dev, "%s - post-scan setup failed\n",
				__func__);
			return ret;
		}
	}

	data->cur_firm_rev = ssp_get_firmware_rev(data);
	dev_info(&data->spi->dev, "MCU Firm Rev : New = %8u\n",
		 data->cur_firm_rev);

	if (!info->dump_mode)
		return 0;

	return ssp_command(data, SSP_MSG2SSP_AP_MCU_DUMP_CHECK, 0);
}

/*
 * sensorhub can request its reinitialization as some brutal and rare error
 * handling. It can be requested from the MCU.
 */
static void ssp_refresh_task(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
					     struct ssp_data, work_refresh);

	dev_info(&data->spi->dev, "refreshing\n");

	data->reset_cnt++;

	if (ssp_initialize_mcu(data) >= 0) {
		ssp_sync_available_sensors(data);
		if (data->last_ap_state != 0)
			ssp_command(data, data->last_ap_state, 0);

		if (data->last_resume_state != 0)
			ssp_command(data, data->last_resume_state, 0);

		data->timeout_cnt = 0;
		data->com_fail_cnt = 0;
	}
}

int ssp_queue_ssp_refresh_task(struct ssp_data *data, unsigned int delay)
{
	cancel_delayed_work_sync(&data->work_refresh);

	return queue_delayed_work(system_power_efficient_wq,
				  &data->work_refresh,
				  msecs_to_jiffies(delay));
}

static const struct of_device_id ssp_of_match[] = {
	{
		.compatible	= "samsung,sensorhub-a3y17lte",
		.data		= &ssp_a3y17lte_info,
	}, {
		.compatible	= "samsung,sensorhub-rinato",
		.data		= &ssp_rinato_info,
	}, {
		.compatible	= "samsung,sensorhub-thermostat",
		.data		= &ssp_thermostat_info,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ssp_of_match);

static const struct spi_device_id ssp_id[] = {
	{ "sensorhub-a3y17lte", (kernel_ulong_t)&ssp_a3y17lte_info },
	{ "sensorhub-rinato", (kernel_ulong_t)&ssp_rinato_info },
	{ "sensorhub-thermostat", (kernel_ulong_t)&ssp_thermostat_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ssp_id);

static struct ssp_data *ssp_parse_dt(struct device *dev)
{
	struct ssp_data *data;

	data = devm_kzalloc_obj(dev, *data);
	if (!data)
		return NULL;

	data->mcu_ap_gpiod = devm_gpiod_get(dev, "mcu-ap", GPIOD_IN);
	if (IS_ERR(data->mcu_ap_gpiod))
		return NULL;

	data->ap_mcu_gpiod = devm_gpiod_get(dev, "ap-mcu", GPIOD_OUT_HIGH);
	if (IS_ERR(data->ap_mcu_gpiod))
		return NULL;

	data->mcu_reset_gpiod = devm_gpiod_get(dev, "mcu-reset",
					       GPIOD_OUT_HIGH);
	if (IS_ERR(data->mcu_reset_gpiod))
		return NULL;

	/* keep the MCU out of its ROM bootloader */
	data->boot0_gpiod = devm_gpiod_get_optional(dev, "boot0",
						    GPIOD_OUT_LOW);
	if (IS_ERR(data->boot0_gpiod))
		return NULL;

	device_property_read_u32(dev, "samsung,accel-position",
				 &data->accel_position);
	device_property_read_u32(dev, "samsung,mag-position",
				 &data->mag_position);
	data->has_light_coef =
		!device_property_read_u32_array(dev,
						"samsung,light-coefficients",
						(u32 *)data->light_coef,
						SSP_LIGHT_COEF_COUNT);
	data->has_prox_thresh =
		!device_property_read_u32_array(dev,
						"samsung,proximity-thresholds",
						data->prox_thresh,
						SSP_PROX_THRESH_COUNT);

	data->sensorhub_info = spi_get_device_match_data(to_spi_device(dev));

	dev_set_drvdata(dev, data);

	return data;
}

struct ssp_consumer {
	struct ssp_data *data;
	enum ssp_sensor_type type;
};

static void ssp_unregister_consumer(void *arg)
{
	struct ssp_consumer *consumer = arg;
	struct ssp_data *data = consumer->data;

	data->sensor_devs[consumer->type] = NULL;
	/* let a running interrupt thread finish with the device */
	synchronize_irq(data->spi->irq);
}

/**
 * ssp_register_consumer() - registers iio consumer in ssp framework
 *
 * @indio_dev:	consumer iio device
 * @type:	ssp sensor type
 */
int ssp_register_consumer(struct iio_dev *indio_dev, enum ssp_sensor_type type)
{
	struct device *dev = indio_dev->dev.parent;
	struct ssp_data *data = dev_get_drvdata(dev->parent);
	struct ssp_consumer *consumer;

	consumer = devm_kzalloc(dev, sizeof(*consumer), GFP_KERNEL);
	if (!consumer)
		return -ENOMEM;

	consumer->data = data;
	consumer->type = type;
	data->sensor_devs[type] = indio_dev;

	return devm_add_action_or_reset(dev, ssp_unregister_consumer, consumer);
}
EXPORT_SYMBOL_NS(ssp_register_consumer, "IIO_SSP_SENSORS");

static int ssp_probe(struct spi_device *spi)
{
	const struct ssp_sensorhub_info *info;
	int ret, i;
	struct ssp_data *data;

	data = ssp_parse_dt(&spi->dev);
	if (!data) {
		dev_err(&spi->dev, "Failed to find platform data\n");
		return -ENODEV;
	}
	info = data->sensorhub_info;

	ret = mfd_add_devices(&spi->dev, PLATFORM_DEVID_NONE,
			      info->cells ?: sensorhub_sensor_devs,
			      info->cells ? info->num_cells :
			      ARRAY_SIZE(sensorhub_sensor_devs), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "mfd add devices fail\n");
		return ret;
	}

	spi->mode = SPI_MODE_1;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup spi\n");
		goto err_setup_spi;
	}

	data->fw_dl_state = SSP_FW_DL_STATE_NONE;
	data->spi = spi;
	spi_set_drvdata(spi, data);

	mutex_init(&data->comm_lock);

	for (i = 0; i < SSP_SENSOR_MAX; ++i) {
		data->delay_buf[i] = SSP_DEFAULT_POLLING_DELAY;
		data->batch_latency_buf[i] = 0;
		data->batch_opt_buf[i] = 0;
		data->check_status[i] = SSP_INITIALIZATION_STATE;
	}

	data->delay_buf[SSP_BIO_HRM_LIB] = 100;

	data->time_syncing = true;

	mutex_init(&data->pending_lock);
	INIT_LIST_HEAD(&data->pending_list);

	atomic_set(&data->enable_refcount, 0);

	INIT_WORK(&data->work_wdt, ssp_wdt_work_func);
	INIT_DELAYED_WORK(&data->work_refresh, ssp_refresh_task);

	timer_setup(&data->wdt_timer, ssp_wdt_timer_func, 0);

	ret = request_threaded_irq(data->spi->irq, NULL,
				   ssp_irq_thread_fn,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "SSP_Int", data);
	if (ret < 0) {
		dev_err(&spi->dev, "Irq request fail\n");
		goto err_setup_irq;
	}

	/* Let's start with enabled one so irq balance could be ok */
	data->shut_down = false;

	/* just to avoid unbalanced irq set wake up */
	enable_irq_wake(data->spi->irq);

	if (data->sensorhub_info->boot_time) {
		ssp_toggle_mcu_reset_gpio(data);
		msleep(data->sensorhub_info->boot_time);
	}

	data->fw_dl_state = ssp_check_fwbl(data);
	if (data->fw_dl_state == SSP_FW_DL_STATE_NONE) {
		ret = ssp_initialize_mcu(data);
		if (ret < 0) {
			dev_err(&spi->dev, "Initialize_mcu failed\n");
			goto err_read_reg;
		}
	} else {
		dev_err(&spi->dev, "Firmware version not supported\n");
		ret = -EPERM;
		goto err_read_reg;
	}

	data->initialized = true;

	return 0;

err_read_reg:
	free_irq(data->spi->irq, data);
err_setup_irq:
	mutex_destroy(&data->pending_lock);
	mutex_destroy(&data->comm_lock);
err_setup_spi:
	mfd_remove_devices(&spi->dev);

	dev_err(&spi->dev, "Probe failed!\n");

	return ret;
}

static void ssp_remove(struct spi_device *spi)
{
	struct ssp_data *data = spi_get_drvdata(spi);

	if (ssp_command(data, SSP_MSG2SSP_AP_STATUS_SHUTDOWN, 0) < 0)
		dev_err(&data->spi->dev,
			"SSP_MSG2SSP_AP_STATUS_SHUTDOWN failed\n");

	ssp_enable_mcu(data, false);
	ssp_disable_wdt_timer(data);

	ssp_clean_pending_list(data);

	free_irq(data->spi->irq, data);
	cancel_delayed_work_sync(&data->work_refresh);

	timer_delete_sync(&data->wdt_timer);
	cancel_work_sync(&data->work_wdt);

	mutex_destroy(&data->comm_lock);
	mutex_destroy(&data->pending_lock);

	mfd_remove_devices(&spi->dev);
}

static int ssp_suspend(struct device *dev)
{
	int ret;
	struct ssp_data *data = spi_get_drvdata(to_spi_device(dev));

	data->last_resume_state = SSP_MSG2SSP_AP_STATUS_SUSPEND;

	if (atomic_read(&data->enable_refcount) > 0)
		ssp_disable_wdt_timer(data);

	ret = ssp_command(data, SSP_MSG2SSP_AP_STATUS_SUSPEND, 0);
	if (ret < 0) {
		dev_err(&data->spi->dev,
			"%s SSP_MSG2SSP_AP_STATUS_SUSPEND failed\n", __func__);

		ssp_enable_wdt_timer(data);
		return ret;
	}

	data->time_syncing = false;
	disable_irq(data->spi->irq);

	return 0;
}

static int ssp_resume(struct device *dev)
{
	int ret;
	struct ssp_data *data = spi_get_drvdata(to_spi_device(dev));

	enable_irq(data->spi->irq);

	if (atomic_read(&data->enable_refcount) > 0)
		ssp_enable_wdt_timer(data);

	ret = ssp_command(data, SSP_MSG2SSP_AP_STATUS_RESUME, 0);
	if (ret < 0) {
		dev_err(&data->spi->dev,
			"%s SSP_MSG2SSP_AP_STATUS_RESUME failed\n", __func__);
		ssp_disable_wdt_timer(data);
		return ret;
	}

	/* timesyncing is set by MCU */
	data->last_resume_state = SSP_MSG2SSP_AP_STATUS_RESUME;

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ssp_pm_ops, ssp_suspend, ssp_resume);

static struct spi_driver ssp_driver = {
	.probe = ssp_probe,
	.remove = ssp_remove,
	.id_table = ssp_id,
	.driver = {
		.pm = pm_sleep_ptr(&ssp_pm_ops),
		.of_match_table = ssp_of_match,
		.name = "sensorhub"
	},
};

module_spi_driver(ssp_driver);

MODULE_DESCRIPTION("ssp sensorhub driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
