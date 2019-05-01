/*
 * TLV2553 12-bit ADC driver
 *
 * Copyright (c) 2019 Jonas Larsson <jonas.larsson@systemrefine.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * Datasheet: http://www.ti.com/lit/ds/symlink/tlv2553.pdf
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/regulator/consumer.h>


#include <linux/delay.h>
enum {
	tlv2553,
};

enum tlv2553_command {
	SELECT_ADC_0		= 0x0,
	SELECT_ADC_1		= 0x1,
	SELECT_ADC_2		= 0x2,
	SELECT_ADC_3		= 0x3,
	SELECT_ADC_4		= 0x4,
	SELECT_ADC_5		= 0x5,
	SELECT_ADC_6		= 0x6,
	SELECT_ADC_7		= 0x7,
	SELECT_ADC_8		= 0x8,
	SELECT_ADC_9		= 0x9,
	SELECT_ADC_10		= 0xA,
	SELECT_ADC_REF_AVG	= 0xB,
	SELECT_ADC_REF_NEG	= 0xC,
	SELECT_ADC_REF_POS	= 0XD,
};

struct tlv2553 {
	struct spi_device *spi;
	unsigned int id;
	/* conversion clock */
	struct clk *cclk;
	/* positive analog voltage reference */
	struct regulator *vref_p;
	/* negative analog voltage reference */
	struct regulator *vref_n;
	struct mutex lock;
	struct completion complete;
	/* The number of cclk periods for the S/H's acquisition time */
	unsigned int acquisition_time;

	u8 tx_buf[2] ____cacheline_aligned;
	u8 rx_buf[2];
};

#define TLV2553_VOLTAGE_CHANNEL(chan)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = chan,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE)	\
					| BIT(IIO_CHAN_INFO_OFFSET),	\
		.scan_index = chan,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 13,					\
			.storagebits = 16,				\
			.shift = 3,					\
			.endianness = IIO_BE,				\
		},							\
	}

static const struct iio_chan_spec tlv2553_channels[] = {
	TLV2553_VOLTAGE_CHANNEL(0),
	TLV2553_VOLTAGE_CHANNEL(1),
	TLV2553_VOLTAGE_CHANNEL(2),
	TLV2553_VOLTAGE_CHANNEL(3),
	TLV2553_VOLTAGE_CHANNEL(4),
	TLV2553_VOLTAGE_CHANNEL(5),
	TLV2553_VOLTAGE_CHANNEL(6),
	TLV2553_VOLTAGE_CHANNEL(7),
	TLV2553_VOLTAGE_CHANNEL(8),
	TLV2553_VOLTAGE_CHANNEL(9),
	TLV2553_VOLTAGE_CHANNEL(10),
	IIO_CHAN_SOFT_TIMESTAMP(11),
};

static int tlv2553_wait_eoc(struct tlv2553 *tlvdata, unsigned long timeout)
{
	if (!wait_for_completion_timeout(&tlvdata->complete, timeout))
		return -ETIMEDOUT;

	return 0;
}

static int tlv2553_read_adc(struct tlv2553 *tlvdata, u8 cmd, u16 *value)
{
	struct spi_device *spi = tlvdata->spi;
	int ret;
	u8 tx_buf[2] ____cacheline_aligned;
	u8 rx_buf[2];

	tx_buf[0] = ((cmd & 0x0F) << 4) | 0x0C;
	tx_buf[1] = 0;
	rx_buf[0] = 0;
	rx_buf[1] = 0;

	ret = spi_write(spi, tx_buf, 2);
	if (ret) {
		dev_err(&spi->dev, "tlv2553_read_adc: spi_write failed\n");
		return ret;	
	}

	ret = tlv2553_wait_eoc(tlvdata, msecs_to_jiffies(100));
	if (ret) {
		dev_err(&spi->dev, "tlv2553_read_adc: No response from chip, timeout waiting for EOC\n");
		return ret;
	}

	ret = spi_read(spi, rx_buf, 2);
	if (ret) {
		dev_err(&spi->dev, "tlv2553_read_adc: spi_read failed\n");
		return ret;	
	}

	*value = (u16)rx_buf[0] << 4 | (u16)rx_buf[1] >> 4;
	
	return 0;
}

static void tlv2553_dump_registers(struct tlv2553 *tlvdata)
{
	u16 values[14];
	int ret;
	int i;
	printk("Dumping TLV2553 registers\n");
	for (i = 0; i <= 0xD; i++) {
		ret = tlv2553_read_adc(tlvdata, i, &values[i]);
		if (ret) return;
	}
	for (i = 0; i <= 0xA; i++) {
		printk("    Analog input channel %-2i  : 0x%03x\n", i, values[i]);
	}
	printk("    Test (V_REF+ + V_REF-)/2 : 0x%03x\n", values[0xB]);
	printk("    Test V_REF-              : 0x%03x\n", values[0xC]);
	printk("    Test V_REF+              : 0x%03x\n", values[0xD]);
}

static int tlv2553_read_raw(struct iio_dev *iio,
			     struct iio_chan_spec const *channel, int *value,
			     int *shift, long mask)
{
	return -EINVAL;
}

static irqreturn_t tlv2553_eoc_handler(int irq, void *p)
{
  	struct iio_dev *indio_dev = p;
	struct tlv2553 *tlvdata = iio_priv(indio_dev);

//	printk("tlv2553_eoc_handler\n");
	complete(&tlvdata->complete);

	return IRQ_HANDLED;
}

static const struct iio_info tlv2553_info = {
	.read_raw = tlv2553_read_raw,
	.driver_module = THIS_MODULE,
};

static int tlv2553_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	unsigned long irq_flags;
	struct tlv2553 *tlvdata;
	int ret;
	u16 value = 0;

	printk("tlv2553_probe\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*tlvdata));
	if (!indio_dev) {
	  dev_err(&spi->dev, "Unable to allocate IIO device\n");
		return -ENOMEM;
	}

	tlvdata = iio_priv(indio_dev);
	tlvdata->spi = spi;
	tlvdata->id = spi_get_device_id(spi)->driver_data;
	mutex_init(&tlvdata->lock);
	init_completion(&tlvdata->complete);

	irq_flags = irq_get_trigger_type(spi->irq);
	if (irq_flags != IRQF_TRIGGER_RISING) {
		dev_warn(&spi->dev, "IRQ is not configured as IRQ_TYPE_EDGE_RISING, overriding device tree settings.\n");
	}

	ret = devm_request_irq(&spi->dev, spi->irq, tlv2553_eoc_handler,
			       IRQF_TRIGGER_RISING, indio_dev->name, indio_dev);

	if (ret) {
		dev_err(&spi->dev, "Unable to request ADC IRQ (End of conversion pin)\n");
		return ret;
	}

	spi_set_drvdata(spi, indio_dev);

#if 0
	/* Test that we can read the expected value from V_REF+ (0xFFF) */
	ret = tlv2553_read_adc(tlvdata, SELECT_ADC_REF_POS, &value);
	if (ret) {
		dev_err(&spi->dev, "Unable to read from ADC\n");
		return ret;
	}

	if (value != 0xFFF) {
		dev_err(&spi->dev, "Unexpected response from V_REF+ test, got 0x%x\n", value);
		return -EINVAL;
	}

	/* Test that we can read the expected value from V_REF- (0) */
	ret = tlv2553_read_adc(tlvdata, SELECT_ADC_REF_NEG, &value);
	if (ret) {
		dev_err(&spi->dev, "Unable to read from ADC\n");
		return ret;
	}

	if (value != 0) {
		dev_err(&spi->dev, "Unexpected response from V_REF- test, got 0x%x\n", value);
		return -EINVAL;
	}
#endif
	
	tlv2553_dump_registers(tlvdata);

	return 0;
}

static int tlv2553_remove(struct spi_device *spi)
{
  	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	//struct tlv2553 *tlvdata = iio_priv(indio_dev);

	printk("tlv2553_remove\n");
	
	//	devm_iio_device_free(indio_dev);

	return 0;
}

#ifdef CONFIG_OF

static const struct of_device_id tlv2553_dt_ids[] = {
	{ .compatible = "ti,tlv2553", },
	{}
};
MODULE_DEVICE_TABLE(of, tlv2553_dt_ids);

#endif

static const struct spi_device_id tlv2553_id[] = {
	{ "tlv2553", tlv2553 },
	{}
};
MODULE_DEVICE_TABLE(spi, tlv2553_id);

static struct spi_driver tlv2553_driver = {
	.driver = {
		.name = "tlv2553",
		.of_match_table = of_match_ptr(tlv2553_dt_ids),
	},
	.probe = tlv2553_probe,
	.remove = tlv2553_remove,
	.id_table = tlv2553_id,
};
module_spi_driver(tlv2553_driver);

MODULE_AUTHOR("Jonas Larsson <jonas.larsson@systemrefine.com>");
MODULE_DESCRIPTION("TLV2553");
MODULE_LICENSE("GPL v2");
