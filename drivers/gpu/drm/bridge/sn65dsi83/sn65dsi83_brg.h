/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 CopuLab Ltd.
 */

#ifndef _SN65DSI83_BRG_H__
#define _SN65DSI83_BRG_H__

#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <video/videomode.h>

struct sn65dsi83_brg;
struct sn65dsi83_brg_funcs {
	int (*power_on)(struct sn65dsi83_brg *sn65dsi8383_brg);
	void (*power_off)(struct sn65dsi83_brg *sn65dsi8383_brg);
	int (*reset)(struct sn65dsi83_brg *sn65dsi8383_brg);
	int (*setup)(struct sn65dsi83_brg *sn65dsi8383_brg);
	int (*start_stream)(struct sn65dsi83_brg *sn65dsi8383_brg);
	void (*stop_stream)(struct sn65dsi83_brg *sn65dsi8383_brg);
};

struct sn65dsi83_brg {
	struct i2c_client *client;
	struct gpio_desc *gpio_enable;
	/* Bridge Panel Parameters */
	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
	u32 format;
	u32 mipi_bpp;
	u32 lvds_bpp;

	u8 num_dsi_lanes;
	bool pulse_mode;
	bool de_neg_polarity;
	bool test_mode;

	struct sn65dsi83_brg_funcs *funcs;
};
struct sn65dsi83_brg *sn65dsi83_brg_get(void);

irqreturn_t sn65dsi83_irq_handler(int unused, void *data);

#endif /* _SN65DSI83_BRG_H__ */
