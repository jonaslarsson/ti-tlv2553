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

enum {
	tlv2553,
};

static int tlv2553_probe(struct spi_device *spi)
{
	return -ENOMEM;
}

static int tlv2553_remove(struct spi_device *spi)
{
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
