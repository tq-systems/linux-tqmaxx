// SPDX-License-Identifier: GPL-2.0

static int vc_mipi_ov9281m_cfg(struct vc_sensor const *sensor,
			       unsigned int *out_mode)
{
	struct v4l2_mbus_framefmt const	*fmt = &sensor->current_format;
	unsigned int			mode = 0;

	switch (fmt->code) {
	case MEDIA_BUS_FMT_Y8_1X8:
		mode |= BIT(0);
		break;
	case MEDIA_BUS_FMT_Y10_1X10:
		/* bit 0 is already clear */
		break;
	default:
		v4l2_warn(&sensor->sd, "unsupported OV9281M fmt %04x\n",
			  fmt->code);
		return -EINVAL;
	}

	if (sensor->ext_trig)
		mode |= BIT(1);

	switch (sensor->num_lanes) {
	case 2:
		break;
	default:
		v4l2_warn(&sensor->sd, "unsupported OV9281M num-lanes %d\n",
			  sensor->num_lanes);
		return -EINVAL;
	}

	if (out_mode)
		*out_mode = mode;

	return 0;
}

static struct vc_sensor_reg const VC_OV9281_START[] = {
	{0x0100, 0x01},
};

static struct vc_sensor_reg const VC_OV9281_STOP[] = {
	{0x0100, 0x00},
};

struct vc_sensor_frame_size const VC_OV9281M_FRAME_SIZES[] = {
	{ 1280, 800, MEDIA_BUS_FMT_Y8_1X8 },
	{ 1280, 800, MEDIA_BUS_FMT_Y10_1X10 },
};

static struct vc_sensor_info const VC_OV9281M = {
	.mbus_codes		= VC_MBUS_Y8_10,
	.num_mbus_codes		= ARRAY_SIZE(VC_MBUS_Y8_10),
	.frame_sizes		= VC_OV9281M_FRAME_SIZES,
	.num_frame_sizes	= ARRAY_SIZE(VC_OV9281M_FRAME_SIZES),

	.fn_cfg			= vc_mipi_ov9281m_cfg,

	.seq_start		= {
		._type_tbl	= VC_SEQ_TABLE,
		.tbl		= VC_OV9281_START,
		.tbl_sz		= ARRAY_SIZE(VC_OV9281_START),
	},

	.seq_stop		= {
		._type_tbl	= VC_SEQ_TABLE,
		.tbl		= VC_OV9281_STOP,
		.tbl_sz		= ARRAY_SIZE(VC_OV9281_STOP),
	},
};
