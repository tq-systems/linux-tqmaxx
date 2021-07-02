// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>

#include "vc-fpga.h"

#define SUSPEND_DELAY		(1000)

struct vc_sensor_frame_size {
	unsigned int			width;
	unsigned int			height;
	uint32_t			code;
};

struct vc_sensor_reg {
	uint16_t			addr;
	uint8_t				val;
	uint8_t				delay_ms;
};

enum vc_sensor_seq_type {
	VC_SEQ_NONE,
	VC_SEQ_TABLE,
	VC_SEQ_FN,
};

struct vc_sensor;
union vc_sensor_seq {
	enum vc_sensor_seq_type type;

	struct {
		enum vc_sensor_seq_type _type_tbl;
		struct vc_sensor_reg const *tbl;
		size_t tbl_sz;
	};

	struct {
		enum vc_sensor_seq_type _type_fn;
		int (*fn)(struct vc_sensor *sensor);
	};
};

struct vc_sensor_info {
	uint32_t const *mbus_codes;
	size_t num_mbus_codes;

	struct vc_sensor_frame_size const *frame_sizes;
	size_t num_frame_sizes;

	struct v4l2_ctrl_config const *ctrls;
	size_t num_ctrls;

	int (*fn_cfg)(struct vc_sensor const *sensor,
		      unsigned int *res);

	union vc_sensor_seq		seq_start;
	union vc_sensor_seq		seq_stop;
	union vc_sensor_seq		seq_set_mode;
};

struct vc_sensor {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct vc_fpga			*fpga;
	struct i2c_client		*i2c;
	struct vc_rom_info const	*rom;
	struct v4l2_ctrl_handler	ctrls;
	struct regulator		*reg_vdd;
	struct reset_control		*reset;

	bool				is_streaming;
	bool				is_poweron;

	struct mutex			v4l_lock;
	struct v4l2_mbus_framefmt	current_format;
	bool				ext_trig;

	unsigned int			num_lanes;
	struct vc_sensor_info const	*info;
};

#define sd_to_sensor(_sd) \
	container_of((_sd), struct vc_sensor, sd)

static int vc_mipi_write(struct vc_sensor *sensor, uint16_t addr, uint8_t data)
{
	uint8_t			tx[] = {
		[0] = (addr >> 8) & 0xff,
		[1] = (addr >> 0) & 0xff,
		[2] = data,
	};

	struct i2c_msg		msg[] = {
		[0] = {
			.addr	= sensor->i2c->addr,
			.buf	= tx,
			.len	= sizeof(tx),
		}
	};

	int			rc;

	rc = i2c_transfer(sensor->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (rc < 0) {
		dev_err(&sensor->i2c->dev, "i2c xfer failed: %d\n", rc);
		goto out;
	}

	rc = 0;

out:
	return rc;
}

static int vc_mipi_write_table(struct vc_sensor *sensor,
			       struct vc_sensor_reg const tbl[], size_t len)
{
	size_t		i;
	int		rc = 0;

	for (i = 0; i < len; ++i) {
		unsigned int	delay = tbl[i].delay_ms;

		rc = vc_mipi_write(sensor, tbl[i].addr, tbl[i].val);
		if (rc < 0)
			break;

		if (delay > 20)
			mdelay(delay);
		else if (delay > 0)
			usleep_range(delay * 1000, delay * 1300);
	}

	return rc;
}

static int _vc_mipi_s_power_on(struct vc_sensor *sensor)
{
	struct vc_sensor_info const	*info = sensor->info;
	int				rc;
	unsigned int			cfg_code;

	rc = info->fn_cfg(sensor, &cfg_code);
	if (rc >= 0)
		vc_fpga_cfg(sensor->fpga, cfg_code);

	if (IS_ENABLED(CONFIG_PM))
		rc = pm_runtime_get_sync(&sensor->i2c->dev);
	else
		rc = regulator_enable(sensor->reg_vdd);

	if (rc < 0) {
		dev_warn(&sensor->i2c->dev, "resume failed: %d\n", rc);
		goto out;
	}

	sensor->is_poweron = true;
	rc = 0;

out:
	return rc;
}

static int _vc_mipi_s_power_off(struct vc_sensor *sensor)
{
	int		rc;

	if (IS_ENABLED(CONFIG_PM)) {
		pm_runtime_put_noidle(&sensor->i2c->dev);
		rc = pm_schedule_suspend(&sensor->i2c->dev, SUSPEND_DELAY);
		if (rc == -EAGAIN)
			/* EAGAIN means there are other users */
			rc = 0;
	} else {
		rc = regulator_disable(sensor->reg_vdd);
	}

	if (rc < 0) {
		dev_warn(&sensor->i2c->dev, "suspend failed: %d\n", rc);
		goto out;
	}

	sensor->is_poweron = false;
	rc = 0;

out:
	return rc;
}

static int vc_mipi_s_power(struct v4l2_subdev *sd, int on)
{
	struct vc_sensor		*sensor = sd_to_sensor(sd);
	int				rc;

	mutex_lock(&sensor->v4l_lock);
	if (on && !sensor->is_poweron) {
		rc = _vc_mipi_s_power_on(sensor);
	} else if (!on && sensor->is_poweron) {
		rc = _vc_mipi_s_power_off(sensor);
	} else {
		v4l2_warn(sd, "unsymmetric s_power(%d)\n", on);
		rc = 0;		/* TODO: or -EINVAL? */
	}
	mutex_unlock(&sensor->v4l_lock);

	return rc;
}

static int vc_mipi_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct vc_sensor		*sensor = sd_to_sensor(sd);

	if (code->index >= sensor->info->num_mbus_codes)
		return -EINVAL;

	code->code = sensor->info->mbus_codes[code->index];

	return 0;
}

static int vc_mipi_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct vc_sensor		*sensor = sd_to_sensor(sd);
	unsigned int			index = fse->index;

	struct vc_sensor_frame_size const	*sizes;
	size_t					num_sizes;
	struct vc_sensor_frame_size const	*res_size = NULL;

	sizes = sensor->info->frame_sizes;
	num_sizes = sensor->info->num_frame_sizes;

	while (num_sizes > 0 && !res_size) {
		if (sizes->code != fse->code)
			; /* noop */
		else if (index > 0)
			--index;
		else
			res_size = sizes;

		--num_sizes;
	}

	if (!res_size)
		return -EINVAL;

	fse->max_height = res_size->height;
	fse->min_height = res_size->height;
	fse->max_width  = res_size->width;
	fse->min_width  = res_size->width;

	return 0;
}

static int vc_mipi_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct vc_sensor		*sensor = sd_to_sensor(sd);
	struct v4l2_mbus_framefmt const	*framefmt;

	if (fmt->pad > 0) {
		dev_dbg(sd->dev, "invalid pad %d\n", fmt->pad);
		return -EINVAL;
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		framefmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						      fmt->pad);
	else
		framefmt = &sensor->current_format;

	fmt->format = *framefmt;

	return 0;
}

static int vc_mipi_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct vc_sensor		*sensor = sd_to_sensor(sd);
	size_t				i;

	struct vc_sensor_frame_size const	*new_fmt = NULL;
	struct v4l2_mbus_framefmt		*framefmt;

	/* TODO: iterate through info->frame_sizes and select amatching one */

	for (i = 0; i < sensor->info->num_frame_sizes; ++i) {
		struct vc_sensor_frame_size const	*fs =
			&sensor->info->frame_sizes[i];

		if (fs->width == fmt->format.width &&
		    fs->height == fmt->format.height &&
		    fs->code == fmt->format.code) {
			new_fmt = fs;
			break;
		}
	}

	if (!new_fmt) {
		v4l2_warn(sd, "%s: no matching format found for %dx%d@%04x\n",
			  __func__, fmt->format.width, fmt->format.height,
			  fmt->format.code);
		return -EINVAL;
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(&sensor->sd, cfg,
						      fmt->pad);
		*framefmt = sensor->current_format;
	} else {
		framefmt = &sensor->current_format;
	}

	framefmt->width  = new_fmt->width;
	framefmt->height = new_fmt->height;
	framefmt->code   = new_fmt->code;

	fmt->format = *framefmt;

	return 0;
}

static int vc_mipi_seq(struct vc_sensor *sensor,
		       union vc_sensor_seq const *seq)
{
	BUILD_BUG_ON(&seq->type != &seq->_type_fn);
	BUILD_BUG_ON(&seq->type != &seq->_type_tbl);

	switch (seq->type) {
	case VC_SEQ_NONE:
		return 0;
	case VC_SEQ_TABLE:
		return vc_mipi_write_table(sensor, seq->tbl, seq->tbl_sz);
	case VC_SEQ_FN:
		return seq->fn(sensor);
	}

	WARN_ON(1);
	return -EINVAL;
}

static int _vc_mipi_s_stream_on(struct vc_sensor *sensor)
{
	struct vc_sensor_info const	*info = sensor->info;
	int				rc;
	unsigned int			cfg_code;

	rc = info->fn_cfg(sensor, &cfg_code);
	if (rc < 0)
		goto out;

	if (vc_fpga_cfg(sensor->fpga, cfg_code) == VC_FPGA_CFG_NEED_RESET) {
		rc = reset_control_reset(sensor->reset);
		if (rc < 0)
			goto out;
	}

	rc = v4l2_ctrl_handler_setup(sensor->sd.ctrl_handler);
	if (rc < 0)
		goto out;

	rc = vc_mipi_seq(sensor, &info->seq_set_mode);
	if (rc < 0)
		goto out;

	rc = vc_mipi_seq(sensor, &info->seq_start);
	if (rc < 0)
		goto out;

	sensor->is_streaming = true;
	rc = 0;

out:
	return rc;
}

static int _vc_mipi_s_stream_off(struct vc_sensor *sensor)
{
	struct vc_sensor_info const	*info = sensor->info;
	int				rc;

	rc = vc_mipi_seq(sensor, &info->seq_stop);
	if (rc < 0)
		goto out;

	sensor->is_streaming = false;
	rc = 0;

out:
	return rc;
}

static int vc_mipi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vc_sensor		*sensor = sd_to_sensor(sd);
	int				rc;

	mutex_lock(&sensor->v4l_lock);
	if (!enable && sensor->is_streaming) {
		rc = _vc_mipi_s_stream_off(sensor);
	} else if (enable && !sensor->is_streaming) {
		rc = _vc_mipi_s_stream_on(sensor);
	} else {
		v4l2_warn(sd, "unsymmetric s_stream(%d)\n", enable);
		rc = 0;		/* TODO: or -EINVAL? */
	}
	mutex_unlock(&sensor->v4l_lock);

	return rc;
}

static void vc_mipi_sd_release(struct v4l2_subdev *sd)
{
	struct vc_sensor		*sensor = sd_to_sensor(sd);

	_vc_mipi_s_power_off(sensor);
}

static struct v4l2_subdev_core_ops const vc_mipi_core_ops = {
	.s_power	= vc_mipi_s_power,
};

static struct v4l2_subdev_pad_ops const vc_mipi_pad_ops = {
	.enum_mbus_code		= vc_mipi_enum_mbus_code,
	.enum_frame_size	= vc_mipi_enum_frame_size,
	.get_fmt		= vc_mipi_get_fmt,
	.set_fmt		= vc_mipi_set_fmt,
};

static struct v4l2_subdev_video_ops const vc_mipi_video_ops = {
	.s_stream		= vc_mipi_s_stream,
};

static struct v4l2_subdev_ops const vc_mipi_subdev_ops = {
	.pad		= &vc_mipi_pad_ops,
	.core		= &vc_mipi_core_ops,
	.video		= &vc_mipi_video_ops,
};

static struct v4l2_subdev_internal_ops const vc_mipi_internal_ops = {
	.release	= vc_mipi_sd_release,
};

static struct media_entity_operations const vc_mipi_entity_ops = {
	/* TODO */
};

static int vc_mipi_pm_power_on(struct device *dev)
{
	struct v4l2_subdev		*sd = dev_get_drvdata(dev);
	struct vc_sensor		*sensor = sd_to_sensor(sd);
	struct vc_sensor_info const	*info = sensor->info;
	int				rc;
	unsigned int			cfg_code;

	rc = info->fn_cfg(sensor, &cfg_code);
	if (rc >= 0)
		vc_fpga_cfg(sensor->fpga, cfg_code);

	rc = regulator_enable(sensor->reg_vdd);
	if (rc < 0)
		goto out;

	reset_control_reset(sensor->reset);

	rc = 0;

out:
	return rc;
}


static int vc_mipi_pm_power_off(struct device *dev)
{
	struct v4l2_subdev	*sd = dev_get_drvdata(dev);
	struct vc_sensor	*sensor = sd_to_sensor(sd);

	return regulator_disable(sensor->reg_vdd);
}

static struct dev_pm_ops const vc_mipi_pm_ops = {
	SET_RUNTIME_PM_OPS(vc_mipi_pm_power_off,
			   vc_mipi_pm_power_on,
			   NULL)
};

#include "vc-mipi_generic.inc.c"
#include "vc-mipi_ov9281.inc.c"
#include "vc-mipi_imx327.inc.c"

static int vc_mipi_init_fpga(struct vc_sensor *sensor)
{
	struct device			*dev = &sensor->i2c->dev;
	struct vc_fpga			*fpga = NULL;
	struct vc_rom_info const	*rom = NULL;
	int				rc;

	fpga = of_vc_fpga_find(dev->of_node, "fpga", 0);
	if (IS_ERR(fpga)) {
		rc = PTR_ERR(fpga);
		if (rc == -EPROBE_DEFER)
			dev_dbg(dev, "fpga not available yet\n");
		else
			dev_err(dev, "fpga not available: %d\n", rc);

		goto out;
	}

	rom = vc_fpga_rom_info(fpga);

	if (rom->is_color) {
		switch (rom->sensor) {
		case VC_ROM_SENSOR_IMX327:
			sensor->info = &VC_IMX327C;
			break;
		default:
			sensor->info = NULL;
			break;
		}
	} else {
		switch (rom->sensor) {
		case VC_ROM_SENSOR_OV9281:
			sensor->info = &VC_OV9281M;
			break;
		default:
			sensor->info = NULL;
			break;
		}
	}

	if (!sensor->info) {
		dev_err(dev, "unsupported sensor\n");
		rc = -EINVAL;
		goto out;
	}

	sensor->rom = rom;
	sensor->fpga = fpga;

	fpga = NULL;
	rc = 0;

out:
	vc_fpga_put(fpga);

	return rc;
}

static int vc_mipi_init_v4l2_of(struct vc_sensor *sensor)
{
	struct device			*dev = &sensor->i2c->dev;
	struct fwnode_handle		*endpoint;
	struct v4l2_fwnode_endpoint	ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int				rc;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "missing endpoint\n");
		return -EINVAL;
	}

	rc = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (rc < 0) {
		dev_err(dev, "bad endpoint: %d\n", rc);
		goto out;
	}

	sensor->num_lanes = ep.bus.mipi_csi2.num_data_lanes;
	switch (sensor->num_lanes) {
	case 1:
	case 2:
	case 4:
		break;
	default:
		dev_err(dev, "unsuuproted number of lanes: %d\n",
			sensor->num_lanes);
		rc = -EINVAL;
		break;
	}

	dev_dbg(dev, "num-lanes: %d\n", sensor->num_lanes);

	rc = 0;

out:
	v4l2_fwnode_endpoint_free(&ep);

	return rc;
}

static int vc_mipi_init_v4l2_api(struct vc_sensor *sensor)
{
	struct device			*dev = &sensor->i2c->dev;
	struct vc_sensor_info const	*info = sensor->info;
	int				rc;
	size_t				i;

	sensor->current_format = (struct v4l2_mbus_framefmt) {
		.width		= sensor->info->frame_sizes[0].width,
		.height		= sensor->info->frame_sizes[0].height,
		.code		= sensor->info->frame_sizes[0].code,

		/* TODO: put these information into 'sensor->info' */
		.field		= V4L2_FIELD_NONE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.ycbcr_enc	= V4L2_YCBCR_ENC_DEFAULT,
		.quantization	= V4L2_QUANTIZATION_DEFAULT,
		.xfer_func	= V4L2_XFER_FUNC_DEFAULT,
	};

	v4l2_i2c_subdev_init(&sensor->sd, sensor->i2c, &vc_mipi_subdev_ops);

	v4l2_ctrl_handler_init(&sensor->ctrls, info->num_ctrls);
	sensor->sd.ctrl_handler = &sensor->ctrls;

	rc = sensor->ctrls.error;
	if (rc < 0) {
		dev_err(dev, "failed to initialize ctrls: %d\n", rc);
		goto free_ctrl;
	}

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.ops = &vc_mipi_entity_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.internal_ops = &vc_mipi_internal_ops;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	rc = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (rc < 0) {
		dev_err(dev, "media_entity_pads_init() failed: %d\n",
			rc);
		goto free_ctrl;
	}

	for (i = 0; i < info->num_ctrls; ++i) {
		struct v4l2_ctrl_config const	*ctrl = &info->ctrls[i];
		struct v4l2_ctrl		*cptr;

		cptr = v4l2_ctrl_new_custom(&sensor->ctrls, ctrl, sensor);
		rc = sensor->ctrls.error;
		if (rc < 0) {
			v4l2_warn(&sensor->sd,
				  "failed to register control '%s' (%x): %d\n",
				  ctrl->name ? ctrl->name : v4l2_ctrl_get_name(ctrl->id),
				  ctrl->id, rc);
			break;
		}
	}
	if (rc < 0)
		goto free_entity;

	rc = v4l2_async_register_subdev(&sensor->sd);
	if (rc < 0) {
		dev_err(dev, "Could not register v4l2 device\n");
		goto free_entity;
	}

	rc = 0;

out:
	return rc;

free_entity:
	media_entity_cleanup(&sensor->sd.entity);

free_ctrl:
	v4l2_ctrl_handler_free(&sensor->ctrls);

	goto out;
}

static int vc_mipi_init_dtree(struct vc_sensor *sensor)
{
	struct device		*dev = &sensor->i2c->dev;
	int			rc;

	sensor->reg_vdd = devm_regulator_get(dev, "vdd");
	rc = PTR_ERR_OR_ZERO(sensor->reg_vdd);
	if (rc < 0) {
		if (rc == -EPROBE_DEFER)
			dev_dbg(dev, "vdd not available yet: %d\n", rc);
		else
			dev_err(dev, "can not get vdd supply: %d\n", rc);

		goto out;
	}

	sensor->reset = devm_reset_control_get_exclusive(dev, "sensor");
	rc = PTR_ERR_OR_ZERO(sensor->reset);
	if (rc < 0) {
		if (rc == -EPROBE_DEFER)
			dev_dbg(dev, "reset not available yet: %d\n", rc);
		else
			dev_err(dev, "can not get reset controller: %d\n", rc);

		goto out;
	}

out:
	return rc;
}

static int vc_mipi_i2c_probe(struct i2c_client *i2c,
			     struct i2c_device_id const *devid)
{
	struct device		*dev = &i2c->dev;
	struct vc_sensor	*sensor;
	int			rc;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c = i2c;
	mutex_init(&sensor->v4l_lock);

	rc = vc_mipi_init_dtree(sensor);
	if (rc < 0)
		goto out;

	rc = vc_mipi_init_fpga(sensor);
	if (rc < 0)
		goto out;

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	rc = vc_mipi_init_v4l2_of(sensor);
	if (rc < 0)
		goto out;

	rc = vc_mipi_init_v4l2_api(sensor);
	if (rc < 0)
		goto out;

	i2c_set_clientdata(i2c, sensor);

	rc = 0;

out:
	if (rc < 0)
		vc_fpga_put(sensor->fpga);

	return rc;
}

static int vc_mipi_i2c_remove(struct i2c_client *i2c)
{
	struct vc_sensor	*sensor = i2c_get_clientdata(i2c);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);

	v4l2_ctrl_handler_free(&sensor->ctrls);

	vc_fpga_put(sensor->fpga);

	mutex_destroy(&sensor->v4l_lock);

	return 0;
}

static struct of_device_id const vc_mipi_of_match[] = {
	{.compatible = "vc,mipi-cam" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vc_mipi_of_match);

static struct i2c_device_id const vc_mipi_ids[] = {
	{"vc_mipi", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, vc_mipi_ids);

static struct i2c_driver		vc_mipi_i2c_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "vc_mipi",
		.pm		= &vc_mipi_pm_ops,
		.of_match_table	= of_match_ptr(vc_mipi_of_match),
	},

	.probe =	vc_mipi_i2c_probe,
	.remove =	vc_mipi_i2c_remove,
	.id_table =	vc_mipi_ids,
};
module_i2c_driver(vc_mipi_i2c_driver);

MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
MODULE_DESCRIPTION("VC MIPI Camera Driver");
MODULE_LICENSE("GPL");
