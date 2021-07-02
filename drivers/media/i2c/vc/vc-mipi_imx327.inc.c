// SPDX-License-Identifier: GPL-2.0

static int vc_mipi_imx327_cfg(struct vc_sensor const *sensor,
			      unsigned int *out_mode)
{
	struct v4l2_mbus_framefmt const	*fmt = &sensor->current_format;
	unsigned int			mode = 0;

	switch (fmt->code) {
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		break;
	default:
		v4l2_warn(&sensor->sd, "unsupported IMX327C fmt %04x\n",
			  fmt->code);
		return -EINVAL;
	}

	if (sensor->ext_trig) {
		v4l2_warn(&sensor->sd, "ext trig not supported on IMX327C\n");
		return -EINVAL;
	}

	switch (sensor->num_lanes) {
	case 2:
		break;
	case 4:
		mode |= BIT(0);
		break;
	default:
		v4l2_warn(&sensor->sd, "unsupported IMX327 num-lanes %d\n",
			  sensor->num_lanes);
		return -EINVAL;
	}

	if (out_mode)
		*out_mode = mode;

	return 0;
}

static int vc_mipi_imx327_set_mode(struct vc_sensor *sensor)
{
	struct v4l2_mbus_framefmt const	*fmt = &sensor->current_format;

	struct vc_sensor_reg const regs[] = {
		{ 0x3001, 0x01 },	/* standby */

		{ 0x3472, (fmt->width  >> 0) & 0xff },
		{ 0x3473, (fmt->width  >> 8) & 0xff },
		{ 0x3418, (fmt->height >> 0) & 0xff },
		{ 0x3419, (fmt->height >> 8) & 0xff },
	};

	return vc_mipi_write_table(sensor, regs, ARRAY_SIZE(regs));
}

static int vc_mipi_imx327_set_blklvl(struct vc_sensor *sensor)
{
	struct v4l2_mbus_framefmt const	*fmt = &sensor->current_format;
	uint16_t			v;
	int				rc;

	switch (fmt->code) {
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		v = 0x3c;
		break;
	case MEDIA_BUS_FMT_Y12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		v = 0xf0;
		break;
	default:
		return -EINVAL;
	}

	rc = vc_mipi_write(sensor, 0x300a, (v >> 0) & 0xff);
	if (rc >= 0)
		rc = vc_mipi_write(sensor, 0x300b, (v >> 8) & 0xff);

	return rc;
}

static int vc_mipi_imx327_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vc_sensor		*sensor = ctrl->priv;
	int				rc;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&sensor->i2c->dev))
		return 0;

	rc = vc_mipi_write(sensor, 0x3001, 0x01); /* REGHOLD */
	if (rc < 0)
		goto out;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		rc = vc_mipi_write(sensor, 0x3014, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE: {
		uint32_t			v = ctrl->val;
		struct vc_sensor_reg const regs[] = {
			{ 0x3020, (v >>  0) & 0xff },
			{ 0x3021, (v >>  8) & 0xff },
			{ 0x3022, (v >> 16) & 0x03 },
		};
		rc = vc_mipi_write_table(sensor, regs, ARRAY_SIZE(regs));
		break;
	}

	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val) {
			struct vc_sensor_reg const regs[] = {
				{ 0x300a, 0x00,  0 },
				{ 0x300b, 0x00, 10 },
				{ 0x308c, (BIT(0) | BIT(1) |
					   (ctrl->val << 4)) },
			};

			rc = vc_mipi_write_table(sensor, regs,
						 ARRAY_SIZE(regs));
		} else {
			rc = vc_mipi_write(sensor, 0x308c, 0);
			if (rc >= 0)
				usleep_range(10000, 11000);
			if (rc >= 0)
				rc = vc_mipi_imx327_set_blklvl(sensor);
		}
		break;

	default:
		rc = -ENOTTY;
		break;
	}

	if (rc < 0)
		goto out;

	rc = vc_mipi_write(sensor, 0x3001, 0x00); /* REGHOLD */
	if (rc < 0)
		goto out;

	rc = 0;

out:
	pm_runtime_put_noidle(&sensor->i2c->dev);
	pm_schedule_suspend(&sensor->i2c->dev, SUSPEND_DELAY);

	return rc;
}

static char const * const vc_mipi_imx327_test_pattern_menu[] = {
	"Disabled",
	"Sequence Pattern 1",
	"Horizontal Color-bar Chart",
	"Vertical Color-bar Chart",
	"Sequence Pattern 2",
	"Gradation Pattern 1",
	"Gradation Pattern 2",
	"000/555h Toggle Pattern",
};

static struct v4l2_ctrl_ops const vc_mipi_imx327_ctrl_ops = {
	.s_ctrl			= vc_mipi_imx327_s_ctrl,
};

static struct v4l2_ctrl_config const VC_IMX327_CTRLS[] = {
	{
		.ops		= &vc_mipi_imx327_ctrl_ops,
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
		.min		= 0,
		.max		= 240,
		.def		= 0,
		.step		= 1,
	}, {
		.ops		= &vc_mipi_imx327_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
		.min		= 29,
		.max		= 7767184,
		.def		= 10000,
		.step		= 1,
	}, {
		.ops		= &vc_mipi_imx327_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.qmenu		= vc_mipi_imx327_test_pattern_menu,
		.min		= 0,
		.max		= ARRAY_SIZE(vc_mipi_imx327_test_pattern_menu) - 1,
	},
};

static struct vc_sensor_reg const VC_IMX327_START[] = {
	{0x3000, 0x00, 30},
	{0x3002, 0x00,  0},
};

static struct vc_sensor_reg const VC_IMX327_STOP[] = {
	{0x3002, 0x01, 30},
	{0x3000, 0x01,  0},
};

struct vc_sensor_frame_size const VC_IMX327C_FRAME_SIZES[] = {
	{ 1920, 1080, MEDIA_BUS_FMT_SRGGB10_1X10 },
	{ 1920, 1080, MEDIA_BUS_FMT_SRGGB12_1X12 },
};

static struct vc_sensor_info const VC_IMX327C = {
	.mbus_codes		= VC_MBUS_SRGGB10_12,
	.num_mbus_codes		= ARRAY_SIZE(VC_MBUS_SRGGB10_12),
	.frame_sizes		= VC_IMX327C_FRAME_SIZES,
	.num_frame_sizes	= ARRAY_SIZE(VC_IMX327C_FRAME_SIZES),
	.ctrls			= VC_IMX327_CTRLS,
	.num_ctrls		= ARRAY_SIZE(VC_IMX327_CTRLS),

	.fn_cfg			= vc_mipi_imx327_cfg,

	.seq_start		= {
		._type_tbl	= VC_SEQ_TABLE,
		.tbl		= VC_IMX327_START,
		.tbl_sz		= ARRAY_SIZE(VC_IMX327_START),
	},

	.seq_stop		= {
		._type_tbl	= VC_SEQ_TABLE,
		.tbl		= VC_IMX327_STOP,
		.tbl_sz		= ARRAY_SIZE(VC_IMX327_STOP),
	},

	.seq_set_mode		= {
		._type_fn	= VC_SEQ_FN,
		.fn		= vc_mipi_imx327_set_mode,
	},
};
