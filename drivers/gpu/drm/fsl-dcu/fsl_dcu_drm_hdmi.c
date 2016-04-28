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
#include <linux/of_graph.h>
#include <video/videomode.h>
#include <video/of_display_timing.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>

#include "fsl_dcu_drm_drv.h"
#include "fsl_dcu_drm_output.h"

#define to_drm_encoder_slave(e) \
	container_of(e, struct drm_encoder_slave, base)
#define to_slave_funcs(e)	(to_drm_encoder_slave(e)->slave_funcs)
static void fsl_dcu_drm_hdmienc_mode_set(struct drm_encoder *encoder,
					 struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	const struct drm_encoder_slave_funcs *sfuncs = to_slave_funcs(encoder);

	if (sfuncs->mode_set)
		sfuncs->mode_set(encoder, mode, adjusted_mode);

}

static int
fsl_dcu_drm_hdmienc_atomic_check(struct drm_encoder *encoder,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	return 0;
}

static void fsl_dcu_drm_hdmienc_disable(struct drm_encoder *encoder)
{
	const struct drm_encoder_slave_funcs *sfuncs = to_slave_funcs(encoder);

	if (sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void fsl_dcu_drm_hdmienc_enable(struct drm_encoder *encoder)
{
	const struct drm_encoder_slave_funcs *sfuncs = to_slave_funcs(encoder);

	if (sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_ON);
}

static const struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.atomic_check = fsl_dcu_drm_hdmienc_atomic_check,
	.disable = fsl_dcu_drm_hdmienc_disable,
	.enable = fsl_dcu_drm_hdmienc_enable,
	.mode_set = fsl_dcu_drm_hdmienc_mode_set,
};

static void fsl_dcu_drm_hdmienc_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = fsl_dcu_drm_hdmienc_destroy,
};
int fsl_dcu_drm_hdmienc_create(struct fsl_dcu_drm_device *fsl_dev,
			       struct drm_crtc *crtc)
{
	struct drm_i2c_encoder_driver *driver;
	struct drm_encoder_slave *enc_slave;
	struct drm_encoder *encoder;
	struct i2c_client *i2c_slave;
	int ret;
	struct device_node *entity;
	struct of_endpoint *ep;
	struct device_node *ep_node;
	struct device_node *parent = fsl_dev->dev->of_node;

	ep = devm_kzalloc(fsl_dev->dev, sizeof(struct of_endpoint), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep_node = devm_kzalloc(fsl_dev->dev,
					sizeof(struct device_node), GFP_KERNEL);
	if (!ep_node)
		return -ENOMEM;

	ep_node = of_graph_get_next_endpoint(parent, NULL);

	ret = of_graph_parse_endpoint(ep_node, ep);
	if (ret < 0) {
		of_node_put(ep_node);
		return ret;
	}

	entity = of_graph_get_remote_port_parent(ep->local_node);
	if (!entity) {
		dev_err(fsl_dev->dev, "unconnected endpoint %s, skipping\n",
			ep->local_node->full_name);
		return -ENODEV;
	}
	i2c_slave = of_find_i2c_device_by_node(entity);

	if (!i2c_slave || !i2c_get_clientdata(i2c_slave))
		return -EPROBE_DEFER;

	enc_slave = devm_kzalloc(fsl_dev->dev, sizeof(*enc_slave), GFP_KERNEL);
	if (!enc_slave)
		return -ENOMEM;

	/* Initialize the slave encoder. */
	driver = to_drm_i2c_encoder_driver(
					to_i2c_driver(i2c_slave->dev.driver));
	driver->encoder_init(i2c_slave, fsl_dev->drm, enc_slave);

	fsl_dev->slave = enc_slave;
	encoder = &enc_slave->base;


	encoder->possible_crtcs = 1;
	ret = drm_encoder_init(fsl_dev->drm, encoder, &encoder_funcs,
			       DRM_MODE_ENCODER_TMDS);
	if (ret < 0)
		return ret;

	drm_encoder_helper_add(encoder, &encoder_helper_funcs);
	encoder->crtc = crtc;

	return 0;
}

#define to_fsl_dcu_drm_hdmicon(connector) \
	container_of(connector, struct fsl_dcu_drm_hdmicon, connector)

static struct drm_encoder *fsl_dcu_drm_hdmi_find_encoder(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS)
			return encoder;
	}

	return NULL;
}

static void fsl_dcu_drm_hdmicon_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status
fsl_dcu_drm_hdmicon_detect(struct drm_connector *connector, bool force)
{
	struct fsl_dcu_drm_hdmicon *hdmicon = to_fsl_dcu_drm_hdmicon(connector);

	if (hdmicon->status == connector_status_disconnected)
		return connector_status_disconnected;
	else
		return connector_status_connected;
}

static const struct drm_connector_funcs fsl_dcu_drm_connector_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.destroy = fsl_dcu_drm_hdmicon_destroy,
	.detect = fsl_dcu_drm_hdmicon_detect,
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
};

static struct drm_encoder *
fsl_dcu_drm_hdmicon_best_encoder(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;

	return fsl_dcu_drm_hdmi_find_encoder(dev);
}


static int fsl_dcu_drm_hdmicon_get_modes(struct drm_connector *connector)
{
	struct fsl_dcu_drm_hdmicon *con = to_fsl_dcu_drm_hdmicon(connector);
	const struct drm_encoder_slave_funcs *sfuncs = con->slave->slave_funcs;

	if (sfuncs->get_modes == NULL)
		return 0;

	return sfuncs->get_modes(NULL, connector);
}

static int fsl_dcu_drm_hdmicon_mode_valid(struct drm_connector *connector,
					  struct drm_display_mode *mode)
{
	if (mode->hdisplay > 1024)
		return MODE_VIRTUAL_X;
	else if	(mode->vdisplay > 768)
		return MODE_VIRTUAL_Y;

	return MODE_OK;
}

static const struct drm_connector_helper_funcs connector_helper_funcs = {
	.best_encoder = fsl_dcu_drm_hdmicon_best_encoder,
	.get_modes = fsl_dcu_drm_hdmicon_get_modes,
	.mode_valid = fsl_dcu_drm_hdmicon_mode_valid,
};

int fsl_dcu_drm_hdmicon_create(struct fsl_dcu_drm_device *fsl_dev)
{
	struct drm_device *dev = fsl_dev->drm;
	struct fsl_dcu_drm_hdmicon *hdmicon;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	int ret;

	hdmicon = devm_kzalloc(fsl_dev->dev,
			       sizeof(struct fsl_dcu_drm_hdmicon), GFP_KERNEL);
	if (!hdmicon)
		return -ENOMEM;

	hdmicon->slave = fsl_dev->slave;
	connector = &hdmicon->connector;

	connector->display_info.width_mm = 0;
	connector->display_info.height_mm = 0;
	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(fsl_dev->drm, connector,
				 &fsl_dcu_drm_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret < 0)
		return ret;

	connector->dpms = DRM_MODE_DPMS_OFF;
	drm_connector_helper_add(connector, &connector_helper_funcs);
	ret = drm_connector_register(connector);
	if (ret < 0)
		goto err_cleanup;

	encoder = fsl_dcu_drm_hdmi_find_encoder(dev);
	if (!encoder)
		goto err_cleanup;
	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret < 0)
		goto err_sysfs;

	connector->encoder = encoder;

	drm_object_property_set_value
		(&connector->base, fsl_dev->drm->mode_config.dpms_property,
		DRM_MODE_DPMS_OFF);

	return 0;

err_sysfs:
	drm_connector_unregister(connector);
err_cleanup:
	drm_connector_cleanup(connector);
	return ret;
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SII902x DVI/HDMI driver");
MODULE_LICENSE("GPL");