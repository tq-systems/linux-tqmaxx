/*
 * Driver for TI TUSB80xx USB 3.0 Hub Controller
 * Configuration via SMBus.
 *
 * Copyright (c) 2019 TQ Systems GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/nls.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>


#define DRIVER_NAME	"tusb80xx"
#define DRIVER_DESC	"TI USB 3.0 Hub Controller"

struct tusb80xx {
	struct device *dev;
	struct i2c_client *i2c;
	int gpio_reset;
	u8  power_on_time;
};

static void tusb80xx_reset(struct tusb80xx *hub, int state)
{
	if (!gpio_is_valid(hub->gpio_reset))
		return;

	gpio_set_value_cansleep(hub->gpio_reset, state);

	/* wait for hub recovery/stabilization */
	/* >= 3ms reset pulse time */
	/* >= 0.2 usec setup & hold for sampling strap pins */
	if (state)
		usleep_range(3000, 3500);
}

static int tusb80xx_connect(struct tusb80xx *hub)
{
#if 0
	struct device *dev = hub->dev;
	int err;
	char i2c_wb[2];

	i2c_wb[0] = 0x01; /* 1 Byte */
	i2c_wb[1] = 0x01; /* clear cfgActive */
#endif

	/* force reset pulse */
	tusb80xx_reset(hub, 0);
	tusb80xx_reset(hub, 1);
/*
	err = i2c_smbus_write_i2c_block_data(hub->i2c, 0xf8, 2, i2c_wb);
	if (err) {
		dev_err(dev, "attaching hub failed: %d\n", err);
		return err;
	}
*/
	return 0;
}

#ifdef CONFIG_OF
static int tusb80xx_get_ofdata(struct tusb80xx *hub)
{
	struct device *dev = hub->dev;
	struct device_node *np = dev->of_node;
	int err;

	if (!np) {
		dev_err(dev, "failed to get ofdata\n");
		return -ENODEV;
	}

	hub->gpio_reset = of_get_named_gpio(np, "reset-gpios", 0);
	if (hub->gpio_reset == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (gpio_is_valid(hub->gpio_reset)) {
		err = devm_gpio_request_one(dev, hub->gpio_reset,
					    GPIOF_OUT_INIT_LOW,
					    "tusb80xx reset");
		if (err) {
			dev_err(dev,
				"unable to request GPIO %d as reset pin (%d)\n",
				hub->gpio_reset, err);
			return err;
		}
	}

	return 0;
}

static const struct of_device_id tusb80xx_of_match[] = {
	{
		.compatible = "ti,tusb80xx",
	}, {
		.compatible = "ti,tusb8020b",
	}, {
		.compatible = "ti,tusb8041",
	}, {
		.compatible = "ti,tusb8042",
	}, {
		.compatible = "ti,tusb8043",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, tusb80xx_of_match);
#else /* CONFIG_OF */
static int tusb80xx_get_ofdata(struct tusb80xx *hub,
			       struct tusb80xx_data *data)
{
	return 0;
}
#endif /* CONFIG_OF */

static int tusb80xx_probe(struct tusb80xx *hub)
{
	struct device *dev = hub->dev;
	struct device_node *np = dev->of_node;
	int err;

	if (np) {
		err = tusb80xx_get_ofdata(hub);
		if (err) {
			dev_err(dev, "failed to get ofdata: %d\n", err);
			return err;
		}
	}

	err = tusb80xx_connect(hub);
	if (err) {
		dev_err(dev, "Failed to connect hub (%d)\n", err);
		return err;
	}

	dev_info(dev, "Hub probed successfully\n");

	return 0;
}

static int tusb80xx_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct tusb80xx *hub;

	hub = devm_kzalloc(&i2c->dev, sizeof(struct tusb80xx), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	i2c_set_clientdata(i2c, hub);
	hub->dev = &i2c->dev;
	hub->i2c = i2c;

	return tusb80xx_probe(hub);
}

static const struct i2c_device_id tusb80xx_id[] = {
	{ "tusb80xx", 0 },
	{ "tusb8020b", 0 },
	{ "tusb8041", 0 },
	{ "tusb8042", 0 },
	{ "tusb8043", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tusb80xx_id);

static struct i2c_driver tusb80xx_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(tusb80xx_of_match),
	},
	.probe    = tusb80xx_i2c_probe,
	.id_table = tusb80xx_id,
};

module_i2c_driver(tusb80xx_i2c_driver);

MODULE_AUTHOR("Markus Niebel <Markus.Niebel@tq-group.com>");
MODULE_DESCRIPTION("TUSB80xx USB 3.0 Hub Controller Driver");
MODULE_LICENSE("GPL");
