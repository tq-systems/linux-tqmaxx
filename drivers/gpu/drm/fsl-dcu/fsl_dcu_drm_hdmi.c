/*
 * Copyright 2016 NXP Semiconductor, Inc.
 *
 * NXP DCU drm device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/of_graph.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include "fsl_dcu_drm_drv.h"
#include "fsl_dcu_drm_output.h"
#include "fsl_tcon.h"

static void
fsl_dcu_drm_hdmienc_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	/*TODO*/
}

static int
fsl_dcu_drm_hdmienc_atomic_check(struct drm_encoder *encoder,
				 struct drm_crtc_state *crtc_state,
				 struct drm_connector_state *conn_state)
{
	return 0;
}

static void
fsl_dcu_drm_hdmienc_disable(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct fsl_dcu_drm_device *fsl_dev = dev->dev_private;

	if (fsl_dev->tcon)
		fsl_tcon_bypass_disable(fsl_dev->tcon);
}

static void
fsl_dcu_drm_hdmienc_enable(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct fsl_dcu_drm_device *fsl_dev = dev->dev_private;

	if (fsl_dev->tcon)
		fsl_tcon_bypass_enable(fsl_dev->tcon);
}

static const struct
drm_encoder_helper_funcs encoder_helper_funcs = {
	.atomic_check = fsl_dcu_drm_hdmienc_atomic_check,
	.disable = fsl_dcu_drm_hdmienc_disable,
	.enable = fsl_dcu_drm_hdmienc_enable,
	.mode_set = fsl_dcu_drm_hdmienc_mode_set,
};

static void
fsl_dcu_drm_hdmienc_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct
drm_encoder_funcs encoder_funcs = {
	.destroy = fsl_dcu_drm_hdmienc_destroy,
};

int fsl_dcu_drm_hdmienc_create(struct fsl_dcu_drm_device *fsl_dev,
			       struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	int ret;

	encoder = devm_kzalloc(fsl_dev->dev,
				sizeof(struct drm_encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	encoder->possible_crtcs = 1;
	ret = drm_encoder_init(fsl_dev->drm, encoder, &encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret)
		goto fail_encoder;

	drm_encoder_helper_add(encoder, &encoder_helper_funcs);
	encoder->crtc = crtc;

	return 0;

fail_encoder:
	devm_kfree(fsl_dev->dev, encoder);
	return ret;
}

static struct
drm_encoder *fsl_dcu_drm_hdmi_find_encoder(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS)
			return encoder;
	}

	return NULL;
}

static struct
device_node *detect_hdmi_connection(struct fsl_dcu_drm_device *fsl_dev)
{
	struct device_node *remote_port;
	struct of_endpoint *ep;
	struct device_node *ep_node, *ep_graph;
	int ret;
	struct device_node *parent = fsl_dev->dev->of_node;

	ep = devm_kzalloc(fsl_dev->dev,
				sizeof(struct of_endpoint), GFP_KERNEL);
	if (!ep)
		return NULL;

	ep_node = devm_kzalloc(fsl_dev->dev,
				sizeof(struct device_node), GFP_KERNEL);
	if (!ep_node)
		return NULL;

	ep_graph = of_graph_get_next_endpoint(parent, NULL);
	if (!ep_graph)
		goto error;

	ret = of_graph_parse_endpoint(ep_graph, ep);
	if (ret) {
		of_node_put(ep_graph);
		goto error;
	}

	remote_port = of_graph_get_remote_port_parent(ep->local_node);
	if (!remote_port)
		goto error;

	return remote_port;
error:
	devm_kfree(fsl_dev->dev, ep);
	devm_kfree(fsl_dev->dev, ep_node);
	devm_kfree(fsl_dev->dev, ep_graph);
	return NULL;
}

int fsl_dcu_drm_hdmienc_attach_bridge(struct fsl_dcu_drm_device *fsl_dev)
{
	struct drm_device *drm_dev = fsl_dev->drm;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	struct device_node *remote_port;
	int ret;

	remote_port = detect_hdmi_connection(fsl_dev);
	if (!remote_port)
		return -ENODEV;

	bridge = of_drm_find_bridge(remote_port);
	if (!bridge)
		return -ENODEV;

	encoder = fsl_dcu_drm_hdmi_find_encoder(drm_dev);
	if (!encoder)
		return -ENODEV;

	encoder->bridge = bridge;
	bridge->encoder = encoder;

	ret = drm_bridge_attach(encoder, bridge, NULL);
	if (ret)
		goto error;

	return 0;
error:
	encoder->funcs->destroy(encoder);
	return ret;
}

