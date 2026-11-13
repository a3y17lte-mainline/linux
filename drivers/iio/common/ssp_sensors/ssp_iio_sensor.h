/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SSP_IIO_SENSOR_H__
#define __SSP_IIO_SENSOR_H__

#define SSP_CHANNEL_AG_MASK(_type, _mod, _index, _mask) \
{ \
		.type = _type,\
		.modified = 1,\
		.channel2 = _mod,\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),\
		.info_mask_shared_by_type = _mask,\
		.scan_index = _index,\
		.scan_type = {\
			.sign = 's',\
			.realbits = 16,\
			.storagebits = 16,\
			.shift = 0,\
			.endianness = IIO_LE,\
		},\
}

#define SSP_CHANNEL_AG(_type, _mod, _index) \
	SSP_CHANNEL_AG_MASK(_type, _mod, _index, BIT(IIO_CHAN_INFO_SAMP_FREQ))

#define SSP_CHANNEL_AG_SCALED(_type, _mod, _index) \
	SSP_CHANNEL_AG_MASK(_type, _mod, _index, \
			    BIT(IIO_CHAN_INFO_SAMP_FREQ) | BIT(IIO_CHAN_INFO_SCALE))

#define SSP_MS_PER_S			1000
#define SSP_INVERTED_SCALING_FACTOR	1000000U

#define SSP_FACTOR_WITH_MS \
	(SSP_INVERTED_SCALING_FACTOR * SSP_MS_PER_S)

int ssp_common_buffer_postenable(struct iio_dev *indio_dev);

int ssp_common_buffer_postdisable(struct iio_dev *indio_dev);

int ssp_common_process_data(struct iio_dev *indio_dev, void *buf,
			    unsigned int len, int64_t timestamp);

int ssp_common_init(struct iio_dev *indio_dev, enum ssp_sensor_type type,
		    int (*process_data)(struct iio_dev *indio_dev, void *buf,
					int64_t timestamp));

int ssp_common_read_sample(struct iio_dev *indio_dev, void *dst,
			   unsigned int offset, unsigned int len);

int ssp_common_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan, int *val, int *val2,
			long mask);

int ssp_common_write_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan, int val, int val2,
			 long mask);

/* Converts time in ms to frequency */
static inline void ssp_convert_to_freq(u32 time, int *integer_part,
				       int *fractional)
{
	if (time == 0) {
		*fractional = 0;
		*integer_part = 0;
		return;
	}

	*integer_part = SSP_FACTOR_WITH_MS / time;
	*fractional = *integer_part % SSP_INVERTED_SCALING_FACTOR;
	*integer_part = *integer_part / SSP_INVERTED_SCALING_FACTOR;
}

/* Converts frequency to time in ms */
static inline int ssp_convert_to_time(int integer_part, int fractional)
{
	u64 value;

	value = (u64)integer_part * SSP_INVERTED_SCALING_FACTOR + fractional;
	if (value == 0)
		return 0;

	return div64_u64((u64)SSP_FACTOR_WITH_MS, value);
}
/* reads the s16 raw value of an SSP_CHANNEL_AG channel */
static inline int ssp_read_ag_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, int *val)
{
	__le16 raw;
	int ret;

	ret = ssp_common_read_sample(indio_dev, &raw,
				     chan->scan_index * sizeof(raw),
				     sizeof(raw));
	if (ret)
		return ret;

	*val = (s16)le16_to_cpu(raw);

	return IIO_VAL_INT;
}

#endif /* __SSP_IIO_SENSOR_H__ */
