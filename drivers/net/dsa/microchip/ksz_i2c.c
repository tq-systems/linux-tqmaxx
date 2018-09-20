/*
 * Microchip KSZ series register access through I2C
 *
 * change of ksz_spi.c
 * Copyright (C) 2018 Max Merchel, TQ Systems GmbH  <Max.Merchel@tq-group.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <asm/unaligned.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>

#include "ksz_priv.h"

static int ksz_i2c_read_reg(struct i2c_client *i2c, u32 reg, u8 *val,
			    unsigned int len)
{
	/*u16 addr = (u16)reg;*/
	u8 addr[2] = { (u8)((reg & 0xff00) >> 8), (u8)(reg) };
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 2,
			.buf = (u8 *)&addr
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val
		},
	};

	if (!i2c->adapter) {
		dev_err(&i2c->dev, "%s error, no i2c->adapter\n",
			__func__);
		return -ENODEV;
	}

	ret = i2c_transfer(i2c->adapter, msg, 2);

	if (ret < 0) {
		dev_err(&i2c->dev, "%s: Error %d on register transfer\n"
		, __func__, ret);
	} else {
		ret = 0;
	}
	return ret;
}

static int ksz_i2c_read(struct ksz_device *dev, u32 reg, u8 *data,
			unsigned int len)
{
	struct i2c_client *i2c = dev->priv;

	return ksz_i2c_read_reg(i2c, reg, data, len);
}

static int ksz_i2c_read8(struct ksz_device *dev, u32 reg, u8 *val)
{
	return ksz_i2c_read(dev, reg, val, 1);
}

static int ksz_i2c_read16(struct ksz_device *dev, u32 reg, u16 *val)
{
	int ret = ksz_i2c_read(dev, reg, (u8 *)val, 2);

	if (!ret)
		*val = be16_to_cpu(*val);

	return ret;
}

static int ksz_i2c_read24(struct ksz_device *dev, u32 reg, u32 *val)
{
	int ret;

	*val = 0;
	ret = ksz_i2c_read(dev, reg, (u8 *)val, 3);
	if (!ret) {
		*val = be32_to_cpu(*val);
		/* convert to 24bit */
		*val >>= 8;
	}

	return ret;
}

static int ksz_i2c_read32(struct ksz_device *dev, u32 reg, u32 *val)
{
	int ret = ksz_i2c_read(dev, reg, (u8 *)val, 4);

	if (!ret)
		*val = be32_to_cpu(*val);

	return ret;
}

static int ksz_i2c_write_reg(struct i2c_client *i2c, u32 reg, u8 *val,
			     unsigned int len)
{
	int ret;
	u8 buf[(2 + len)];
	int i;

	struct i2c_msg msg = {
		.addr = i2c->addr,
		.flags = 0,
		.len = 2 + len,
		.buf = buf
	};

	buf[0] = (u8)((reg & 0xff00) >> 8);
	buf[1] = (u8)(reg);

	for (i = 0; i < len ; i++)
		buf[2 + i] = *(val + i);

	if (!i2c->adapter) {
		dev_err(&i2c->dev, "%s error, no i2c->adapter\n",
			__func__);
		return -ENODEV;
	}

	ret = i2c_transfer(i2c->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(&i2c->dev, "%s: Error %d on register transfer\n"
			, __func__, ret);
	} else {
		ret = 0;
	}
	return ret;
}

static int ksz_i2c_write8(struct ksz_device *dev, u32 reg, u8 value)
{
	struct i2c_client *i2c = dev->priv;

	return ksz_i2c_write_reg(i2c, reg, &value, 1);
}

static int ksz_i2c_write16(struct ksz_device *dev, u32 reg, u16 value)
{
	struct i2c_client *i2c = dev->priv;

	value = cpu_to_be16(value);
	return ksz_i2c_write_reg(i2c, reg, (u8 *)&value, 2);
}

static int ksz_i2c_write24(struct ksz_device *dev, u32 reg, u32 value)
{
	struct i2c_client *i2c = dev->priv;

	/* make it to big endian 24bit from MSB */
	value <<= 8;
	value = cpu_to_be32(value);
	return ksz_i2c_write_reg(i2c, reg, (u8 *)&value, 3);
}

static int ksz_i2c_write32(struct ksz_device *dev, u32 reg, u32 value)
{
	struct i2c_client *i2c = dev->priv;

	value = cpu_to_be32(value);
	return ksz_i2c_write_reg(i2c, reg, (u8 *)&value, 4);
}

static const struct ksz_io_ops ksz_i2c_ops = {
	.read8 = ksz_i2c_read8,
	.read16 = ksz_i2c_read16,
	.read24 = ksz_i2c_read24,
	.read32 = ksz_i2c_read32,
	.write8 = ksz_i2c_write8,
	.write16 = ksz_i2c_write16,
	.write24 = ksz_i2c_write24,
	.write32 = ksz_i2c_write32,
};

static int ksz_i2c_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *i2c_dev_id)
{
	struct ksz_device *dev;
	int ret;

	dev = ksz_switch_alloc(&i2c->dev, &ksz_i2c_ops, i2c);
	if (!dev)
		return -ENOMEM;

	if (i2c->dev.platform_data)
		dev->pdata = i2c->dev.platform_data;

	ret = ksz_switch_register(dev);
	if (ret)
		return ret;

	i2c_set_clientdata(i2c, dev);

	return 0;
}

static int ksz_i2c_remove(struct i2c_client *i2c)
{
	struct ksz_device *dev = i2c_get_clientdata(i2c);

	if (dev)
		ksz_switch_remove(dev);

	return 0;
}

static const struct of_device_id ksz_dt_ids[] = {
	{ .compatible = "microchip,ksz9477" },
	{ .compatible = "microchip,ksz9897" },
	{},
};
MODULE_DEVICE_TABLE(of, ksz_dt_ids);

static struct i2c_driver ksz_i2c_driver = {
	.driver = {
		.name	= "ksz9477-switch",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ksz_dt_ids),
	},
	.probe	= ksz_i2c_probe,
	.remove	= ksz_i2c_remove,
};

module_i2c_driver(ksz_i2c_driver);

MODULE_AUTHOR("Woojung Huh <Woojung.Huh@microchip.com>");
MODULE_DESCRIPTION("Microchip KSZ Series Switch I2C access Driver");
MODULE_LICENSE("GPL");
