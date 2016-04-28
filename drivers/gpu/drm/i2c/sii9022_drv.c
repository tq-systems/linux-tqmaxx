/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * Freescale DCU drm device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/backlight.h>
#include <video/videomode.h>
#include <video/of_display_timing.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>


#define SII902X_INPUT_BUS_FMT	0x08
#define SII902X_TPI_AVI_INPUT_FMT	0x09
#define SII902X_TPI_AVI_OUTPUT_FMT	0x0A
#define SII902X_SYS_CONTROL	0x1A
#define SII902X_SYS_CTR_DDC_REQ	BIT(2)
#define SII902X_SYS_CTR_DDC_BUS_AVAI	(BIT(2) | BIT(1))
#define SII902X_TPI_FAMILY_DEV_ID	0x1B
#define SII902X_TPI_DEV_REV_ID	0x1C
#define SII902X_TPI_REV_LEVEL_ID	0x1D
#define SII902X_POWER_STATE	0x1E
#define SII902X_TPI_AUDIO_CFG0	0x24
#define SII902X_TPI_AUDIO_CFG1	0x25
#define SII902X_TPI_AUDIO_CFG2	0x26
#define SII902X_TPI_AUDIO_CFG3	0x27
#define SII902X_TPI_HDCP_REV	0x30
#define SII902X_TPI_INT_ENABLE	0x3C
#define SII902X_TPI_INT_STATUS	0x3D
#define SII902X_TPI_INT_PLUG_IN	BIT(2)
#define SII902X_GENERAL_PURPOSE_IO0	0xBC
#define SII902X_GENERAL_PURPOSE_IO1	0xBD
#define SII902X_GENERAL_PURPOSE_IO2	0xBE
#define SII902X_TRANS_MODE_DIFF	0xC7

bool g_enable_hdmi;

struct sii902x_data {
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
} *sii902x;

#define to_sii902x_data(x) \
	((struct sii902x_data *)to_encoder_slave(x)->slave_priv)

static struct i2c_client *sii902x_to_i2c(struct sii902x_data *sii902x)
{
	return sii902x->client;
}

/* HW access functions */
static s32 sii902x_write(const struct i2c_client *client,
			 u8 command, u8 value)
{
	return i2c_smbus_write_byte_data(client, command, value);
}

static s32 sii902x_read(const struct i2c_client *client, u8 command)
{
	int val;

	val = i2c_smbus_read_word_data(client, command);

	return val & 0xff;
}

static void sii902x_poweron(void)
{
	/* Turn on DVI or HDMI */
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, 0x00);
}

static void sii902x_poweroff(void)
{
	/* disable tmds before changing resolution */
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, 0x10);
}


static void sii902x_chip_id(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	/* read device ID */
	val = sii902x_read(client, SII902X_TPI_FAMILY_DEV_ID);
	pr_info("Sii902x: read id = 0x%02X", val);
	val = sii902x_read(client, SII902X_TPI_DEV_REV_ID);
	pr_info("-0x%02X", val);
	val = sii902x_read(client, SII902X_TPI_REV_LEVEL_ID);
	pr_info("-0x%02X", val);
	val = sii902x_read(client, SII902X_TPI_HDCP_REV);
	pr_info("-0x%02X\n", val);
}

static int sii902x_initialize(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int ret, cnt;

	for (cnt = 0; cnt < 5; cnt++) {
		/* Set 902x in hardware TPI mode on and jump out of D3 state */
		ret = sii902x_write(client, SII902X_TRANS_MODE_DIFF, 0x00);
		if (ret < 0)
			break;
	}
	if (ret != 0)
		dev_err(&client->dev, "cound not find device\n");

	return ret;
}

static void sii902x_enable_source(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	sii902x_write(client, SII902X_GENERAL_PURPOSE_IO0, 0x01);
	sii902x_write(client, SII902X_GENERAL_PURPOSE_IO1, 0x82);
	val = sii902x_read(client, SII902X_GENERAL_PURPOSE_IO2);
	val |= 0x1;
	sii902x_write(client, SII902X_GENERAL_PURPOSE_IO2, val);
}

static void sii902x_power_up_tx(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	val = sii902x_read(client, SII902X_POWER_STATE);
	val &= ~0x3;
	sii902x_write(client, SII902X_POWER_STATE, val);
}

static int sii902x_get_edid_preconfig(void)
{
	int old, dat, ret = 0, cnt = 100;

	old = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);

	sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_REQ);
	do {
		cnt--;
		msleep(20);
		dat = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		ret = -1;
		goto done;
	}

	sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_BUS_AVAI);

done:
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, old);
	return ret;
}

static int __init enable_hdmi_setup(char *str)
{
	g_enable_hdmi = true;

	return 1;
}

__setup("hdmi", enable_hdmi_setup);

static irqreturn_t sii902x_detect_handler(int irq, void *data)
{
	if (g_enable_hdmi)
		g_enable_hdmi = false;

	return IRQ_HANDLED;
}

static int sii902x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	int ret, err;

	if (!g_enable_hdmi)
		return -EPERM;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -ENODEV;
	}

	sii902x = devm_kzalloc(&client->dev, sizeof(*sii902x), GFP_KERNEL);
	if (!sii902x)
		return -ENOMEM;

	sii902x->client = client;
	i2c_set_clientdata(client, sii902x);

	err = sii902x_initialize(sii902x);
	if (err)
		return err;

	sii902x_chip_id(sii902x);
	sii902x_power_up_tx(sii902x);
	sii902x_enable_source(sii902x);

	if (sii902x_get_edid_preconfig() < 0)
		dev_warn(&client->dev, "Read edid preconfig failed\n");

	if (client->irq) {
		ret = devm_request_irq(&client->dev, client->irq,
				       sii902x_detect_handler, 0,
				       "SII902x_det", sii902x);
		if (ret < 0)
			dev_warn(&client->dev,
				 "cound not request det irq %d\n",
				 client->irq);
		else {
			/*enable cable hot plug irq*/
			sii902x_write(client, SII902X_TPI_INT_ENABLE, 0x01);
		}
	}

	return 0;
}

static int sii902x_remove(struct i2c_client *client)
{
	sii902x_poweroff();
	return 0;
}

/* DRM encoder functions */

static void
sii902x_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	if (mode == DRM_MODE_DPMS_ON)
		sii902x_poweron();
	else
		sii902x_poweroff();
}


static int
sii902x_encoder_mode_valid(struct drm_encoder *encoder,
			  struct drm_display_mode *mode)
{
	return 0;
}

static void
sii902x_encoder_mode_set(struct drm_encoder *encoder,
			struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode)
{
	struct videomode vm;
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	int i;

	/* Power up */
	sii902x_power_up_tx(sii902x);

	drm_display_mode_to_videomode(mode, &vm);
	data[0] = PICOS2KHZ(vm.pixelclock) / 10;
	data[2] = vm.hsync_len + vm.hback_porch +
		  vm.hactive + vm.hfront_porch;
	data[3] = vm.vsync_len + vm.vfront_porch +
		  vm.vactive + vm.vback_porch;
	refresh = data[2] * data[3];
	refresh = (PICOS2KHZ(vm.pixelclock) * 1000) / refresh;
	data[1] = refresh * 100;

	tmp = (u8 *)data;
	for (i = 0; i < 8; i++)
		sii902x_write(sii902x->client, i, tmp[i]);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	sii902x_write(sii902x->client, SII902X_INPUT_BUS_FMT, 0x70);
	/* Set input format to RGB */
	sii902x_write(sii902x->client, SII902X_TPI_AVI_INPUT_FMT, 0x00);
	/* set output format to RGB */
	sii902x_write(sii902x->client, SII902X_TPI_AVI_OUTPUT_FMT, 0x00);
	/* audio setup */
	sii902x_write(sii902x->client, SII902X_TPI_AUDIO_CFG1, 0x00);
	sii902x_write(sii902x->client, SII902X_TPI_AUDIO_CFG2, 0x40);
	sii902x_write(sii902x->client, SII902X_TPI_AUDIO_CFG3, 0x00);
}

struct edid *sii902x_drm_get_edid(struct drm_connector *connector,
				  struct i2c_adapter *adapter)
{
	int old, dat, cnt = 100;
	struct edid *edid;

	old = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);

	sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_REQ);
	do {
		cnt--;
		msleep(20);
		dat = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		edid = NULL;
		goto done;
	}

	sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_BUS_AVAI);

	/* edid reading */
	edid = drm_get_edid(connector, adapter);

	cnt = 100;
	do {
		cnt--;
		sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
			      old & ~SII902X_SYS_CTR_DDC_BUS_AVAI);
		msleep(20);
		dat = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		edid = NULL;

done:
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, old);
	return edid;
}

static int
sii902x_encoder_get_modes(struct drm_encoder *encoder,
			 struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct i2c_adapter *adap = to_i2c_adapter(sii902x->client->dev.parent);
	struct edid *edid;
	struct drm_display_mode *mode;
	int ret = 0;

	edid = sii902x_drm_get_edid(connector, adap);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	} else {
		dev_dbg(dev->dev, "failed to get edid\n");
	}

	list_for_each_entry(mode, &connector->probed_modes, head) {
		if (mode->hdisplay == 1024 || mode->vdisplay == 768)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		else
			mode->type &= ~DRM_MODE_TYPE_PREFERRED;
	}

	return ret;
}

static struct drm_encoder_slave_funcs sii902x_encoder_funcs = {
	.dpms = sii902x_encoder_dpms,
	.mode_valid = sii902x_encoder_mode_valid,
	.mode_set = sii902x_encoder_mode_set,
	.get_modes = sii902x_encoder_get_modes,
};
static int
sii902x_encoder_init(struct i2c_client *client,
		    struct drm_device *dev,
		    struct drm_encoder_slave *encoder)
{

	encoder->slave_funcs = &sii902x_encoder_funcs;

	return 0;
}
static const struct i2c_device_id sii902x_id[] = {
	{ "sii902x", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sii902x_id);

static const struct of_device_id sii902x_dt_ids[] = {
	{ .compatible = "sil,sii9022a", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sii902x_dt_ids);

static struct drm_i2c_encoder_driver sii902x_driver = {
	.i2c_driver = {
		.probe = sii902x_probe,
		.remove = sii902x_remove,
		.driver = {
			.name = "sii902x",
			.owner = THIS_MODULE,
			.of_match_table = sii902x_dt_ids,
		},
		.id_table = sii902x_id,
	},
	.encoder_init = sii902x_encoder_init,
};

/* Module initialization */

static int __init sii902x_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &sii902x_driver);
}

static void __exit sii902x_exit(void)
{
	drm_i2c_encoder_unregister(&sii902x_driver);
}


MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SII902x DVI/HDMI driver");
MODULE_LICENSE("GPL");


module_init(sii902x_init);
module_exit(sii902x_exit);