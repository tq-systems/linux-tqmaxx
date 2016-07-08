/*
 * Copyright (C) 2012-2014 Mentor Graphics Inc.
 * Copyright 2005-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/bitrev.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/err.h>
#include "ipu-prv.h"

/* IC Register Offsets */
#define IC_CONF                 0x0000
#define IC_PRP_ENC_RSC          0x0004
#define IC_PRP_VF_RSC           0x0008
#define IC_PP_RSC               0x000C
#define IC_CMBP_1               0x0010
#define IC_CMBP_2               0x0014
#define IC_IDMAC_1              0x0018
#define IC_IDMAC_2              0x001C
#define IC_IDMAC_3              0x0020
#define IC_IDMAC_4              0x0024

/* IC Register Fields */
#define IC_CONF_PRPENC_EN       (1 << 0)
#define IC_CONF_PRPENC_CSC1     (1 << 1)
#define IC_CONF_PRPENC_ROT_EN   (1 << 2)
#define IC_CONF_PRPVF_EN        (1 << 8)
#define IC_CONF_PRPVF_CSC1      (1 << 9)
#define IC_CONF_PRPVF_CSC2      (1 << 10)
#define IC_CONF_PRPVF_CMB       (1 << 11)
#define IC_CONF_PRPVF_ROT_EN    (1 << 12)
#define IC_CONF_PP_EN           (1 << 16)
#define IC_CONF_PP_CSC1         (1 << 17)
#define IC_CONF_PP_CSC2         (1 << 18)
#define IC_CONF_PP_CMB          (1 << 19)
#define IC_CONF_PP_ROT_EN       (1 << 20)
#define IC_CONF_IC_GLB_LOC_A    (1 << 28)
#define IC_CONF_KEY_COLOR_EN    (1 << 29)
#define IC_CONF_RWS_EN          (1 << 30)
#define IC_CONF_CSI_MEM_WR_EN   (1 << 31)

#define IC_IDMAC_1_CB0_BURST_16         (1 << 0)
#define IC_IDMAC_1_CB1_BURST_16         (1 << 1)
#define IC_IDMAC_1_CB2_BURST_16         (1 << 2)
#define IC_IDMAC_1_CB3_BURST_16         (1 << 3)
#define IC_IDMAC_1_CB4_BURST_16         (1 << 4)
#define IC_IDMAC_1_CB5_BURST_16         (1 << 5)
#define IC_IDMAC_1_CB6_BURST_16         (1 << 6)
#define IC_IDMAC_1_CB7_BURST_16         (1 << 7)
#define IC_IDMAC_1_PRPENC_ROT_MASK      (0x7 << 11)
#define IC_IDMAC_1_PRPENC_ROT_OFFSET    11
#define IC_IDMAC_1_PRPVF_ROT_MASK       (0x7 << 14)
#define IC_IDMAC_1_PRPVF_ROT_OFFSET     14
#define IC_IDMAC_1_PP_ROT_MASK          (0x7 << 17)
#define IC_IDMAC_1_PP_ROT_OFFSET        17
#define IC_IDMAC_1_PP_FLIP_RS           (1 << 22)
#define IC_IDMAC_1_PRPVF_FLIP_RS        (1 << 21)
#define IC_IDMAC_1_PRPENC_FLIP_RS       (1 << 20)

#define IC_IDMAC_2_PRPENC_HEIGHT_MASK   (0x3ff << 0)
#define IC_IDMAC_2_PRPENC_HEIGHT_OFFSET 0
#define IC_IDMAC_2_PRPVF_HEIGHT_MASK    (0x3ff << 10)
#define IC_IDMAC_2_PRPVF_HEIGHT_OFFSET  10
#define IC_IDMAC_2_PP_HEIGHT_MASK       (0x3ff << 20)
#define IC_IDMAC_2_PP_HEIGHT_OFFSET     20

#define IC_IDMAC_3_PRPENC_WIDTH_MASK    (0x3ff << 0)
#define IC_IDMAC_3_PRPENC_WIDTH_OFFSET  0
#define IC_IDMAC_3_PRPVF_WIDTH_MASK     (0x3ff << 10)
#define IC_IDMAC_3_PRPVF_WIDTH_OFFSET   10
#define IC_IDMAC_3_PP_WIDTH_MASK        (0x3ff << 20)
#define IC_IDMAC_3_PP_WIDTH_OFFSET      20

struct ic_task_regoffs {
	u32 rsc;
	u32 tpmem_csc[2];
};

struct ic_task_bitfields {
	u32 ic_conf_en;
	u32 ic_conf_rot_en;
	u32 ic_conf_cmb_en;
	u32 ic_conf_csc1_en;
	u32 ic_conf_csc2_en;
	u32 ic_cmb_galpha_bit;
};

struct ic_task_channels {
	u8 in;
	u8 out;
	u8 rot_in;
	u8 rot_out;
	u8 in_prev;
	u8 in_next;
};

static const struct ic_task_regoffs ic_task_reg[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER] = {
		.rsc = IC_PRP_ENC_RSC,
		.tpmem_csc = {0x2008, 0},
	},
	[IC_TASK_VIEWFINDER] = {
		.rsc = IC_PRP_VF_RSC,
		.tpmem_csc = {0x4028, 0x4040},
	},
	[IC_TASK_POST_PROCESSOR] = {
		.rsc = IC_PP_RSC,
		.tpmem_csc = {0x6060, 0x6078},
	},
};

static const struct ic_task_bitfields ic_task_bit[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER] = {
		.ic_conf_en = IC_CONF_PRPENC_EN,
		.ic_conf_rot_en = IC_CONF_PRPENC_ROT_EN,
		.ic_conf_cmb_en = 0,    /* NA */
		.ic_conf_csc1_en = IC_CONF_PRPENC_CSC1,
		.ic_conf_csc2_en = 0,   /* NA */
		.ic_cmb_galpha_bit = 0, /* NA */
	},
	[IC_TASK_VIEWFINDER] = {
		.ic_conf_en = IC_CONF_PRPVF_EN,
		.ic_conf_rot_en = IC_CONF_PRPVF_ROT_EN,
		.ic_conf_cmb_en = IC_CONF_PRPVF_CMB,
		.ic_conf_csc1_en = IC_CONF_PRPVF_CSC1,
		.ic_conf_csc2_en = IC_CONF_PRPVF_CSC2,
		.ic_cmb_galpha_bit = 0,
	},
	[IC_TASK_POST_PROCESSOR] = {
		.ic_conf_en = IC_CONF_PP_EN,
		.ic_conf_rot_en = IC_CONF_PP_ROT_EN,
		.ic_conf_cmb_en = IC_CONF_PP_CMB,
		.ic_conf_csc1_en = IC_CONF_PP_CSC1,
		.ic_conf_csc2_en = IC_CONF_PP_CSC2,
		.ic_cmb_galpha_bit = 8,
	},
};

static const struct ic_task_channels ic_task_ch[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER] = {
		.in = IPUV3_CHANNEL_MEM_IC_PRP_VF,
		.out = IPUV3_CHANNEL_IC_PRP_ENC_MEM,
		.rot_in = IPUV3_CHANNEL_MEM_ROT_ENC,
		.rot_out = IPUV3_CHANNEL_ROT_ENC_MEM,
	},
	[IC_TASK_VIEWFINDER] = {
		.in = IPUV3_CHANNEL_MEM_VDI_CUR,
		.out = IPUV3_CHANNEL_IC_PRP_VF_MEM,
		.rot_in = IPUV3_CHANNEL_MEM_ROT_VF,
		.rot_out = IPUV3_CHANNEL_ROT_VF_MEM,
		.in_prev = IPUV3_CHANNEL_MEM_VDI_PREV,
		.in_next = IPUV3_CHANNEL_MEM_VDI_NEXT,
	},
	[IC_TASK_POST_PROCESSOR] = {
		.in = IPUV3_CHANNEL_MEM_IC_PP,
		.out = IPUV3_CHANNEL_IC_PP_MEM,
		.rot_in = IPUV3_CHANNEL_MEM_ROT_PP,
		.rot_out = IPUV3_CHANNEL_ROT_PP_MEM,
	},
};

struct image_convert_ctx {
	void (*complete)(void *ctx, int err);
	void *complete_context;

	struct list_head list;
	struct ipu_image in;
	struct ipu_image in_n;
	struct ipu_image in_p;
	struct ipu_image out;

	void *freep;

	bool rotate:1;

	u32 rsc;
};

struct ipu_ic_priv;

struct ipu_ic {
	enum ipu_ic_task task;
	const struct ic_task_regoffs *reg;
	const struct ic_task_bitfields *bit;
	const struct ic_task_channels *ch;

	enum ipu_color_space in_cs, g_in_cs;
	enum ipu_color_space out_cs;
	bool graphics;
	bool rotation;
	bool in_use;

	struct ipu_ic_priv *priv;

	struct ipuv3_channel *input_channel_p;
	struct ipuv3_channel *input_channel;
	struct ipuv3_channel *input_channel_n;
	struct ipuv3_channel *output_channel;
	struct ipuv3_channel *rotation_input_channel;
	struct ipuv3_channel *rotation_output_channel;

	struct list_head image_list;

	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct completion complete;
};

struct ipu_ic_priv {
	void __iomem *base;
	void __iomem *tpmem_base;
	spinlock_t lock;
	struct ipu_soc *ipu;
	int use_count;
	struct ipu_ic task[IC_NUM_TASKS];
};

static inline u32 ipu_ic_read(struct ipu_ic *ic, unsigned offset)
{
	return readl(ic->priv->base + offset);
}

static inline void ipu_ic_write(struct ipu_ic *ic, u32 value,
				unsigned offset)
{
	writel(value, ic->priv->base + offset);
}

struct ic_csc_params {
	s16 coeff[3][3];	/* signed 9-bit integer coefficients */
	s16 offset[3];		/* signed 11+2-bit fixed point offset */
	u8 scale:2;		/* scale coefficients * 2^(scale-1) */
	bool sat:1;		/* saturate to (16, 235(Y) / 240(U, V)) */
};

/*
 * Y = R *  .299 + G *  .587 + B *  .114;
 * U = R * -.169 + G * -.332 + B *  .500 + 128.;
 * V = R *  .500 + G * -.419 + B * -.0813 + 128.;
 */
static const struct ic_csc_params ic_csc_rgb2ycbcr = {
	.coeff = {
		{ 77, 150, 29 },
		{ 469, 427, 128 },
		{ 128, 405, 491 },
	},
	.offset = { 0, 512, 512 },
	.scale = 1,
};

/* transparent RGB->RGB matrix for graphics combining */
static const struct ic_csc_params ic_csc_rgb2rgb = {
	.coeff = {
		{ 128, 0, 0 },
		{ 0, 128, 0 },
		{ 0, 0, 128 },
	},
	.scale = 2,
};

/*
 * R = (1.164 * (Y - 16)) + (1.596 * (Cr - 128));
 * G = (1.164 * (Y - 16)) - (0.392 * (Cb - 128)) - (0.813 * (Cr - 128));
 * B = (1.164 * (Y - 16)) + (2.017 * (Cb - 128);
 */
static const struct ic_csc_params ic_csc_ycbcr2rgb = {
	.coeff = {
		{ 149, 0, 204 },
		{ 149, 462, 408 },
		{ 149, 255, 0 },
	},
	.offset = { -446, 266, -554 },
	.scale = 2,
};

static int init_csc(struct ipu_ic *ic,
		    enum ipu_color_space inf,
		    enum ipu_color_space outf,
		    int csc_index)
{
	struct ipu_ic_priv *priv = ic->priv;
	const struct ic_csc_params *params;
	u32 __iomem *base;
	const u16 (*c)[3];
	const u16 *a;
	u32 param;

	base = (u32 __iomem *)
		(priv->tpmem_base + ic->reg->tpmem_csc[csc_index]);

	if (inf == IPUV3_COLORSPACE_YUV && outf == IPUV3_COLORSPACE_RGB)
		params = &ic_csc_ycbcr2rgb;
	else if (inf == IPUV3_COLORSPACE_RGB && outf == IPUV3_COLORSPACE_YUV)
		params = &ic_csc_rgb2ycbcr;
	else if (inf == IPUV3_COLORSPACE_RGB && outf == IPUV3_COLORSPACE_RGB)
		params = &ic_csc_rgb2rgb;
	else {
		dev_err(priv->ipu->dev, "Unsupported color space conversion\n");
		return -EINVAL;
	}

	/* Cast to unsigned */
	c = (const u16 (*)[3])params->coeff;
	a = (const u16 *)params->offset;

	param = ((a[0] & 0x1f) << 27) | ((c[0][0] & 0x1ff) << 18) |
		((c[1][1] & 0x1ff) << 9) | (c[2][2] & 0x1ff);
	writel(param, base++);

	param = ((a[0] & 0x1fe0) >> 5) | (params->scale << 8) |
		(params->sat << 9);
	writel(param, base++);

	param = ((a[1] & 0x1f) << 27) | ((c[0][1] & 0x1ff) << 18) |
		((c[1][0] & 0x1ff) << 9) | (c[2][0] & 0x1ff);
	writel(param, base++);

	param = ((a[1] & 0x1fe0) >> 5);
	writel(param, base++);

	param = ((a[2] & 0x1f) << 27) | ((c[0][2] & 0x1ff) << 18) |
		((c[1][2] & 0x1ff) << 9) | (c[2][1] & 0x1ff);
	writel(param, base++);

	param = ((a[2] & 0x1fe0) >> 5);
	writel(param, base++);

	return 0;
}

static int calc_resize_coeffs(struct ipu_ic *ic,
			      u32 in_size, u32 out_size,
			      u32 *resize_coeff,
			      u32 *downsize_coeff)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct ipu_soc *ipu = priv->ipu;
	u32 temp_size, temp_downsize;

	/*
	 * Input size cannot be more than 4096, and output size cannot
	 * be more than 1024
	 */
	if (in_size > 4096) {
		dev_err(ipu->dev, "Unsupported resize (in_size > 4096)\n");
		return -EINVAL;
	}
	if (out_size > 1024) {
		dev_err(ipu->dev, "Unsupported resize (out_size > 1024)\n");
		return -EINVAL;
	}

	/* Cannot downsize more than 4:1 */
	if ((out_size << 2) < in_size) {
		dev_err(ipu->dev, "Unsupported downsize\n");
		return -EINVAL;
	}

	/* Compute downsizing coefficient */
	temp_downsize = 0;
	temp_size = in_size;
	while (((temp_size > 1024) || (temp_size >= out_size * 2)) &&
	       (temp_downsize < 2)) {
		temp_size >>= 1;
		temp_downsize++;
	}
	*downsize_coeff = temp_downsize;

	/*
	 * compute resizing coefficient using the following equation:
	 * resize_coeff = M * (SI - 1) / (SO - 1)
	 * where M = 2^13, SI = input size, SO = output size
	 */
	*resize_coeff = (8192L * (temp_size - 1)) / (out_size - 1);
	if (*resize_coeff >= 16384L) {
		dev_err(ipu->dev, "Warning! Overflow on resize coeff.\n");
		*resize_coeff = 0x3FFF;
	}

	return 0;
}

void ipu_ic_task_enable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 ic_conf;

	spin_lock_irqsave(&priv->lock, flags);

	ic_conf = ipu_ic_read(ic, IC_CONF);

	ic_conf |= ic->bit->ic_conf_en;

	if (ic->rotation)
		ic_conf |= ic->bit->ic_conf_rot_en;

	if (ic->in_cs != ic->out_cs)
		ic_conf |= ic->bit->ic_conf_csc1_en;

	if (ic->graphics) {
		ic_conf |= ic->bit->ic_conf_cmb_en;
		ic_conf |= ic->bit->ic_conf_csc1_en;

		if (ic->g_in_cs != ic->out_cs)
			ic_conf |= ic->bit->ic_conf_csc2_en;
	}

	ipu_ic_write(ic, ic_conf, IC_CONF);

	spin_unlock_irqrestore(&priv->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_ic_task_enable);

void ipu_ic_task_disable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 ic_conf;

	spin_lock_irqsave(&priv->lock, flags);

	ic_conf = ipu_ic_read(ic, IC_CONF);

	ic_conf &= ~(ic->bit->ic_conf_en |
		     ic->bit->ic_conf_csc1_en |
		     ic->bit->ic_conf_rot_en);
	if (ic->bit->ic_conf_csc2_en)
		ic_conf &= ~ic->bit->ic_conf_csc2_en;
	if (ic->bit->ic_conf_cmb_en)
		ic_conf &= ~ic->bit->ic_conf_cmb_en;

	ipu_ic_write(ic, ic_conf, IC_CONF);

	ic->rotation = ic->graphics = false;

	spin_unlock_irqrestore(&priv->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_ic_task_disable);

int ipu_ic_task_graphics_init(struct ipu_ic *ic,
			      enum ipu_color_space in_g_cs,
			      bool galpha_en, u32 galpha,
			      bool colorkey_en, u32 colorkey)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 reg, ic_conf;
	int ret = 0;

	if (ic->task == IC_TASK_ENCODER)
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);

	ic_conf = ipu_ic_read(ic, IC_CONF);

	if (!(ic_conf & ic->bit->ic_conf_csc1_en)) {
		/* need transparent CSC1 conversion */
		ret = init_csc(ic, IPUV3_COLORSPACE_RGB,
			       IPUV3_COLORSPACE_RGB, 0);
		if (ret)
			goto unlock;
	}

	ic->g_in_cs = in_g_cs;

	if (ic->g_in_cs != ic->out_cs) {
		ret = init_csc(ic, ic->g_in_cs, ic->out_cs, 1);
		if (ret)
			goto unlock;
	}

	if (galpha_en) {
		ic_conf |= IC_CONF_IC_GLB_LOC_A;
		reg = ipu_ic_read(ic, IC_CMBP_1);
		reg &= ~(0xff << ic->bit->ic_cmb_galpha_bit);
		reg |= (galpha << ic->bit->ic_cmb_galpha_bit);
		ipu_ic_write(ic, reg, IC_CMBP_1);
	} else
		ic_conf &= ~IC_CONF_IC_GLB_LOC_A;

	if (colorkey_en) {
		ic_conf |= IC_CONF_KEY_COLOR_EN;
		ipu_ic_write(ic, colorkey, IC_CMBP_2);
	} else
		ic_conf &= ~IC_CONF_KEY_COLOR_EN;

	ipu_ic_write(ic, ic_conf, IC_CONF);

	ic->graphics = true;
unlock:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_ic_task_graphics_init);

int ipu_ic_task_init(struct ipu_ic *ic,
		     int in_width, int in_height,
		     int out_width, int out_height,
		     enum ipu_color_space in_cs,
		     enum ipu_color_space out_cs,
		     u32 rsc)
{
	struct ipu_ic_priv *priv = ic->priv;
	u32 downsize_coeff, resize_coeff;
	unsigned long flags;
	int ret = 0;

	if (!rsc) {
		/* Setup vertical resizing */
		ret = calc_resize_coeffs(ic, in_height, out_height,
					 &resize_coeff, &downsize_coeff);
		if (ret)
			return ret;

		rsc = (downsize_coeff << 30) | (resize_coeff << 16);

		/* Setup horizontal resizing */
		ret = calc_resize_coeffs(ic, in_width, out_width,
					 &resize_coeff, &downsize_coeff);
		if (ret)
			return ret;

		rsc |= (downsize_coeff << 14) | resize_coeff;
	}

	spin_lock_irqsave(&priv->lock, flags);

	ipu_ic_write(ic, rsc, ic->reg->rsc);

	/* Setup color space conversion */
	ic->in_cs = in_cs;
	ic->out_cs = out_cs;

	if (ic->in_cs != ic->out_cs) {
		ret = init_csc(ic, ic->in_cs, ic->out_cs, 0);
		if (ret)
			goto unlock;
	}

unlock:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_ic_task_init);

int ipu_ic_task_idma_init(struct ipu_ic *ic, struct ipuv3_channel *channel,
			  u32 width, u32 height, int burst_size,
			  enum ipu_rotate_mode rot)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct ipu_soc *ipu = priv->ipu;
	u32 ic_idmac_1, ic_idmac_2, ic_idmac_3;
	u32 temp_rot = bitrev8(rot) >> 5;
	bool need_hor_flip = false;
	unsigned long flags;
	int ret = 0;

	if ((burst_size != 8) && (burst_size != 16)) {
		dev_err(ipu->dev, "Illegal burst length for IC\n");
		return -EINVAL;
	}

	width--;
	height--;

	if (temp_rot & 0x2)	/* Need horizontal flip */
		need_hor_flip = true;

	spin_lock_irqsave(&priv->lock, flags);

	ic_idmac_1 = ipu_ic_read(ic, IC_IDMAC_1);
	ic_idmac_2 = ipu_ic_read(ic, IC_IDMAC_2);
	ic_idmac_3 = ipu_ic_read(ic, IC_IDMAC_3);

	switch (channel->num) {
	case IPUV3_CHANNEL_IC_PP_MEM:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB2_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB2_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PP_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PP_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PP_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PP_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PP_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PP_WIDTH_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_IC_PP:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB5_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB5_BURST_16;
		break;
	case IPUV3_CHANNEL_MEM_ROT_PP:
		ic_idmac_1 &= ~IC_IDMAC_1_PP_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PP_ROT_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_IC_PRP_VF:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB6_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB6_BURST_16;
		break;
	case IPUV3_CHANNEL_IC_PRP_ENC_MEM:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB0_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB0_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPENC_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPENC_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPENC_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPENC_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPENC_WIDTH_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_ROT_ENC:
		ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPENC_ROT_OFFSET;
		break;
	case IPUV3_CHANNEL_IC_PRP_VF_MEM:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB1_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB1_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPVF_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPVF_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPVF_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPVF_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPVF_WIDTH_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_ROT_VF:
		ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPVF_ROT_OFFSET;
		break;
	case IPUV3_CHANNEL_G_MEM_IC_PRP_VF:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB3_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB3_BURST_16;
		break;
	case IPUV3_CHANNEL_G_MEM_IC_PP:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB4_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB4_BURST_16;
		break;
	case IPUV3_CHANNEL_VDI_MEM_IC_VF:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB7_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB7_BURST_16;
		break;
	default:
		goto unlock;
	}

	ipu_ic_write(ic, ic_idmac_1, IC_IDMAC_1);
	ipu_ic_write(ic, ic_idmac_2, IC_IDMAC_2);
	ipu_ic_write(ic, ic_idmac_3, IC_IDMAC_3);

	if (rot >= IPU_ROTATE_90_RIGHT)
		ic->rotation = true;

unlock:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_ic_task_idma_init);

static struct image_convert_ctx *ipu_image_convert_next(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct ipuv3_channel *ch_in = ic->input_channel;
	struct ipuv3_channel *ch_out = ic->output_channel;
	struct image_convert_ctx *ctx;
	struct ipu_image *in_p, *in, *in_n;
	struct ipu_image *out;
	int ret;
	unsigned long flags;
	unsigned int inburst, outburst;
	unsigned int in_height;

	spin_lock_irqsave(&priv->lock, flags);

	if (list_empty(&ic->image_list)) {
		spin_unlock_irqrestore(&priv->lock, flags);
		return NULL;
	}

	ctx = list_first_entry(&ic->image_list, struct image_convert_ctx, list);

	list_del(&ctx->list);

	spin_unlock_irqrestore(&priv->lock, flags);

	in_p = &ctx->in_p;
	in = &ctx->in;
	in_n = &ctx->in_n;
	out = &ctx->out;

	ipu_cpmem_zero(ch_in);
	ipu_cpmem_zero(ch_out);

	inburst = in->rect.width & 0xf ? 8 : 16;
	outburst = out->rect.width & 0xf ? 8 : 16;

	ipu_ic_enable(ic);

	ipu_ic_task_idma_init(ic, ic->input_channel, in->rect.width,
			      in->rect.height, inburst, IPU_ROTATE_NONE);
	ipu_ic_task_idma_init(ic, ic->output_channel, out->rect.width,
			      out->rect.height, outburst, IPU_ROTATE_NONE);

	ipu_cpmem_set_image(ch_in, &ctx->in);
	ipu_cpmem_set_image(ch_out, &ctx->out);

	ipu_cpmem_set_burstsize(ch_in, inburst);
	ipu_cpmem_set_burstsize(ch_out, outburst);

	in_height = in->rect.height;

	dev_dbg(priv->ipu->dev, "%s: %dx%d(%dx%d@%d,%d) -> %dx%d(%dx%d@%d,%d)\n",
		__func__, in->pix.width, in->pix.height,
		in->rect.width, in->rect.height, in->rect.left, in->rect.top,
		out->pix.width, out->pix.height,
		out->rect.width, out->rect.height,
		out->rect.left, out->rect.top);

	dev_dbg(priv->ipu->dev,
		"%s: hscale: >>%d, *8192/%d vscale: >>%d, *8192/%d\n",
		__func__, (ctx->rsc >> 14) & 0x3, (ctx->rsc & 0x3fff),
		ctx->rsc >> 30, (ctx->rsc >> 16) & 0x3fff);

	ret = ipu_ic_task_init(ic, in->rect.width, in_height,
			out->rect.width, out->rect.height,
			ipu_pixelformat_to_colorspace(in->pix.pixelformat),
			ipu_pixelformat_to_colorspace(out->pix.pixelformat),
			ctx->rsc);
	if (ret) {
		ipu_ic_disable(ic);
		return ERR_PTR(ret);
	}

	ipu_idmac_enable_channel(ic->input_channel);
	ipu_idmac_enable_channel(ic->output_channel);

	ipu_ic_task_enable(ic);

	ipu_idmac_select_buffer(ic->input_channel, 0);
	ipu_idmac_select_buffer(ic->output_channel, 0);

	return ctx;
}

static void ipu_image_convert_work(struct work_struct *work)
{
	struct ipu_ic *ic = container_of(work, struct ipu_ic, work);
	struct image_convert_ctx *ctx;
	int ret;

	while (1) {
		int task_error = 0;

		ctx = ipu_image_convert_next(ic);
		if (!ctx)
			return;

		if (IS_ERR(ctx)) {
			task_error = PTR_ERR(ctx);
		} else {
			ret = wait_for_completion_interruptible_timeout(
						&ic->complete, 100 * HZ);
			if (!ret)
				task_error = -ETIMEDOUT;
		}

		ipu_ic_task_disable(ic);
		ipu_ic_disable(ic);

		if (ctx->complete)
			ctx->complete(ctx->complete_context, task_error);
		kfree(ctx->freep);
	}
}

static irqreturn_t ipu_image_convert_handler(int irq, void *context)
{
	struct ipu_ic *ic = context;

	complete(&ic->complete);

	return IRQ_HANDLED;
}


/*
 * IDMAC base addresses are 8-byte aligned
 */
static int ipu_image_halign(u32 pixfmt)
{
	switch (pixfmt) {
	/* 2 RGB32 pixels correspond to 8 bytes */
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		return 2;
	/* 4 RGB565 or YUYV pixels correspond to 8 bytes */
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUYV:
		return 4;
	/*
	 * 8 RGB24 pixels correspond to 24 bytes,
	 * 8 NV12 pixels correspond to 8 bytes, both in luma and chroma
	 */
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_NV12:
		return 8;
	/* 16 YUV420 pixels correspond to 16 bytes in luma, 8 bytes in chroma */
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YUV422P:
		return 16;
	default:
		return -EINVAL;
	}
}

/*
 * Vertically chroma-subsampled formats are limited to even heights and vertical
 * positions
 */
static int ipu_image_valign(u32 pixfmt)
{
	switch (pixfmt) {
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YUV422P:
		return 1;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		return 2;
	default:
		return -EINVAL;
	}
}

#define round_closest(x, y) round_down((x) + (y)/2, (y))

struct image_convert_ctx *ipu_image_convert_prepare(struct ipu_soc *ipu,
		struct ipu_image *in, struct ipu_image *out,
		enum ipu_image_scale_ctrl ctrl, int *num_tiles)
{
	struct image_convert_ctx *ctx, *c;
	int htiles, vtiles;
	int in_valign, in_halign, in_burst, out_valign, out_halign, out_burst;
	int left, top;
	int x, y;
	int h_resize_opt, v_resize_opt;
	u32 v_downsize_coeff = 0, h_downsize_coeff = 0;
	u32 v_resize_coeff, h_resize_coeff;

	/* validate input */
	if (in->rect.width < 16 || out->rect.width < 16 ||
	    (in->rect.width / 4) > out->rect.width ||
	    (in->rect.height / 4) > out->rect.height)
		return ERR_PTR(-EINVAL);

	/* tile setup */
	htiles = DIV_ROUND_UP(out->rect.width, 1024);
	vtiles = DIV_ROUND_UP(out->rect.height, 1024);

	in_valign = ipu_image_valign(in->pix.pixelformat);
	in_halign = ipu_image_halign(in->pix.pixelformat);
	out_valign = ipu_image_valign(out->pix.pixelformat);
	out_halign = ipu_image_halign(out->pix.pixelformat);

	/* IC bursts are limited to either 8 or 16 pixels */
	in_burst = 8;
	out_burst = 8;

	if (in_valign < 0 || in_halign < 0 ||
	    out_valign < 0 || out_halign < 0) {
		dev_err(ipu->dev, "unsupported in/out format\n");
		return ERR_PTR(-EINVAL);
	}

	/* compute static decimator coefficients */
	while ((in->rect.width >> h_downsize_coeff) > out->rect.width)
		h_downsize_coeff++;
	while ((in->rect.height >> v_downsize_coeff) > out->rect.height)
		v_downsize_coeff++;

	/* move and crop the output image according to IDMAC limitations */
	switch (ctrl) {
	case IPU_IMAGE_SCALE_ROUND_DOWN:
		left = round_up(in->rect.left, in_halign);
		top = round_up(in->rect.top, in_valign);
		in->rect.width = in->rect.width - (left - in->rect.left);
		in->rect.height = in->rect.height - (top - in->rect.top);
		in->rect.left = left;
		in->rect.top = top;
		left = round_up(out->rect.left, out_halign);
		top = round_up(out->rect.top, out_valign);
		out->rect.width = round_down(out->rect.width - (left -
					     out->rect.left), out_burst);
		out->rect.height = round_down(out->rect.height - (top -
					      out->rect.top), out_valign);
		break;
	case IPU_IMAGE_SCALE_ROUND_UP:
		left = round_down(in->rect.left, in_halign);
		top = round_down(in->rect.top, in_valign);
		in->rect.width = in->rect.width + in->rect.left - left;
		in->rect.height = in->rect.height + in->rect.top - top;
		in->rect.left = left;
		in->rect.top = top;
		left = round_down(out->rect.left, out_halign);
		top = round_down(out->rect.top, out_valign);
		out->rect.width = round_up(out->rect.width + out->rect.left -
					   left, out_burst);
		out->rect.height = round_up(out->rect.height + out->rect.top -
					    top, out_valign);
		break;
	case IPU_IMAGE_SCALE_PIXELPERFECT:
		left = round_down(in->rect.left, in_halign);
		top = round_down(in->rect.top, in_valign);
		in->rect.width = in->rect.width + in->rect.left - left;
		in->rect.height = in->rect.height + in->rect.top - top;
		in->rect.left = left;
		in->rect.top = top;
		left = round_down(out->rect.left + out_halign / 2, out_halign);
		top = round_down(out->rect.top + out_valign / 2, out_valign);
		/*
		 * don't round width and height to burst size / pixel format
		 * limitations yet, we do it after determining the scaling
		 * coefficients
		 */
		out->rect.width = out->rect.width + out->rect.left - left;
		out->rect.height = out->rect.height + out->rect.top - top;
		break;
	default:
		return ERR_PTR(-EINVAL);
	}
	out->rect.left = left;
	out->rect.top = top;

	/* Round input width and height according to decimation */
	in->rect.width = round_down(in->rect.width, 1 << h_downsize_coeff);
	in->rect.height = round_down(in->rect.height, 1 << v_downsize_coeff);

	dev_dbg(ipu->dev,
		"%s: in: %dx%d(%dx%d@%d,%d) -> out: %dx%d(%dx%d@%d,%d)\n",
		__func__, in->pix.width, in->pix.height, in->rect.width,
		in->rect.height, in->rect.left, in->rect.top, out->pix.width,
		out->pix.height, out->rect.width, out->rect.height,
		out->rect.left, out->rect.top);

	/*
	 * Compute the bilinear resizing coefficients that can/could be used if
	 * scaling using a single tile. The bottom right pixel should sample the
	 * input as close as possible to but not beyond the bottom right input
	 * pixel out of the decimator:
	 *
	 * (out->rect.width - 1) * h_resize / 8192.0 <= (in->rect.width >>
	 *						 h_downsize_coeff) - 1
	 * (out->rect.height - 1) * v_resize / 8192.0 <= (in->rect.height >>
	 *						  v_downsize_coeff) - 1
	 */
	h_resize_opt = 8192 * ((in->rect.width >> h_downsize_coeff) - 1) /
		       (out->rect.width - 1);
	v_resize_opt = 8192 * ((in->rect.height >> v_downsize_coeff) - 1) /
		       (out->rect.height - 1);

	dev_dbg(ipu->dev,
		"%s: hscale: >>%d, *8192/%d vscale: >>%d, *8192/%d, %dx%d tiles\n",
		__func__, h_downsize_coeff, h_resize_opt, v_downsize_coeff,
		v_resize_opt, htiles, vtiles);

	ctx = kcalloc(htiles * vtiles, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	c = ctx;

	for (x = htiles - 1; x >= 0; x--) {
		int in_right, out_right;

		/*
		 * Since we render tiles right to left, the right edge
		 * is already known. Depending on tile position and
		 * scaling mode, we may overshoot it.
		 */
		if (x == htiles - 1) {
			out_right = out->rect.left + out->rect.width;
			in_right = in->rect.left + in->rect.width;
		} else {
			struct image_convert_ctx *c_right = c - vtiles;

			out_right = c_right->out.rect.left;
			in_right = c_right->in.rect.left;
		}

		/* Now determine the left edge of this tile column */
		if (x == 0) {
			/* For the leftmost column this is trivial */
			c->out.rect.left = out->rect.left;
			c->in.rect.left = in->rect.left;
		} else {
			int best_left, best_in_left;
			int min_left, max_left;
			int min_diff = INT_MAX;

			/*
			 * Find the best possible left edge. It must be adjusted
			 * according to IDMAC limitations, and should be
			 * chosen so that
			 * (in->rect.left + (c->out.rect.left - out->rect.left)
			 *  * h_resize_opt / (8192 >> h_downsize_coeff))
			 * is as close as possible to a valid left edge in the
			 * input.
			 */
			min_left = max(0,
				       round_up(out_right - 1024, out_halign));
			max_left = min(round_down(out_right, out_halign),
				       x * 1024);
			best_left = min_left;
			best_in_left = (best_left - out->rect.left) *
				       h_resize_opt;
			for (left = min_left; left < max_left;
			     left += out_halign) {
				int diff, in_left;

				/*
				 * In ROUND_UP and ROUND_DOWN modes, for the
				 * rightmost column, only consider left edges
				 * that are a multiple of the burst size away
				 * from the right edge.
				 */
				if ((ctrl != IPU_IMAGE_SCALE_PIXELPERFECT) &&
				    (x == htiles - 1) &&
				    ((out_right - left) % out_burst))
					continue;
				in_left = in->rect.left +
					  (((left - out->rect.left) *
					    h_resize_opt) << h_downsize_coeff);
				diff = abs(in_left -
					   round_closest(in_left,
							 8192 * in_halign));

				if (diff < min_diff) {
					min_diff = diff;
					best_left = left;
					best_in_left = in_left;
				}
			}

			c->out.rect.left = best_left;
			c->in.rect.left = DIV_ROUND_CLOSEST(best_in_left, 8192);

			dev_dbg(ipu->dev,
				"%s: tile(%d,y):\tleft: %d -> %d (instead of %d.%04d -> %d)",
				__func__, x, c->in.rect.left,
				c->out.rect.left, best_in_left / 8192,
				(best_in_left % 8192) * 10000 / 8192,
				out->rect.left +
				DIV_ROUND_CLOSEST((c->in.rect.left -
						   in->rect.left) *
						  (8192 >> h_downsize_coeff),
						  h_resize_opt));
		}

		/* Determine tile width from left and right edges */
		c->out.rect.width = out_right - c->out.rect.left;
		c->in.rect.width = in_right - c->in.rect.left;

		/* Now we can determine the actual per-tile scaling factor */
		if (x == htiles - 1) {
			/*
			 * Round down for the right column, since we
			 * don't want to read beyond the right edge.
			 */
			h_resize_coeff = 8192 * ((c->in.rect.width >>
						 h_downsize_coeff) - 1) /
					 (c->out.rect.width - 1);
		} else {
			/*
			 * Round to closest for seams between tiles for
			 * minimal distortion.
			 */
			h_resize_coeff = DIV_ROUND_CLOSEST(8192 *
							   (c->in.rect.width >>
							    h_downsize_coeff),
							   c->out.rect.width);
		}

		/*
		 * With the scaling factor known, round up output width
		 * to burst size. In ROUND_UP and ROUND_DOWN scaling mode
		 * this is a no-op for the right column.
		 */
		c->out.rect.width = round_up(c->out.rect.width, out_burst);

		/*
		 * Calculate input width from the last accessed input pixel
		 * given output width and scaling coefficients. Round to
		 * burst size.
		 */
		c->in.rect.width = (DIV_ROUND_UP((c->out.rect.width - 1) *
						 h_resize_coeff, 8192) + 1)
				   << h_downsize_coeff;
		c->in.rect.width = round_up(c->in.rect.width, in_burst);

		for (y = vtiles - 1; y >= 0; y--) {
			int in_bottom, out_bottom;

			memcpy(&c->in.pix, &in->pix,
			      sizeof(struct v4l2_pix_format));

			if (y == vtiles - 1) {
				out_bottom = out->rect.top + out->rect.height;
				in_bottom = in->rect.top + in->rect.height;
			} else {
				struct image_convert_ctx *c_below = c - 1;

				out_bottom = c_below->out.rect.top;
				in_bottom = c_below->in.rect.top;

				/*
				 * Copy horizontal parameters from the tile
				 * below
				 */
				c->out.rect.left = c_below->out.rect.left;
				c->out.rect.width = c_below->out.rect.width;
				c->in.rect.left = c_below->in.rect.left;
				c->in.rect.width = c_below->in.rect.width;
			}

			if (y == 0) {
				c->out.rect.top = out->rect.top;
				c->in.rect.top = in->rect.top;
			} else {
				int best_top, best_in_top;
				int min_top, max_top;
				int min_diff = INT_MAX;

				/*
				 * Find the best possible top edge. It must be
				 * adjusted according to IDMAC limitations, and
				 * should be chosen so that
				 * (in->rect.top + (c->out.rect.top -
				 *  out->rect.top) * v_resize_opt /
				 * (8192 >> v_downsize_coeff))
				 * is as close as possible to a valid top edge
				 * in the input.
				 */
				min_top = max(0,
					      round_up(out_bottom - 1024,
						       out_valign));
				max_top = min(round_down(out_bottom,
							 out_halign), y * 1024);
				best_top = min_top;
				best_in_top = (best_top - out->rect.top) *
					       v_resize_opt;
				for (top = min_top; top < max_top;
				     top += out_valign) {
					int diff, in_top;

					in_top = in->rect.top +
						 (((top - out->rect.top) *
						   v_resize_opt) <<
						  v_downsize_coeff);
					diff = abs(in_top -
						   round_closest(in_top, 8192 *
								 in_valign));

					if (diff < min_diff) {
						min_diff = diff;
						best_top = top;
						best_in_top = in_top;
					}
				}

				c->out.rect.top = best_top;
				c->in.rect.top = DIV_ROUND_CLOSEST(best_in_top,
								   8192);

				dev_dbg(ipu->dev,
					"%s: tile(%d,%d):\ttop: %d -> %d (instead of %d.%04d -> %d)",
					__func__, x, y, c->in.rect.top,
					c->out.rect.top, best_in_top / 8192,
					(best_in_top % 8192) * 10000 / 8192,
					out->rect.top +
					DIV_ROUND_CLOSEST((c->in.rect.top -
							   in->rect.top) * (8192
							  >> v_downsize_coeff),
							  v_resize_opt));
			}

			/* Determine tile height from top and bottom edges */
			c->out.rect.height = out_bottom - c->out.rect.top;
			c->in.rect.height = in_bottom - c->in.rect.top;

			/*
			 * Now we can determine the actual vertical per-tile
			 * scaling factor
			 */
			if (y == vtiles - 1) {
				/*
				 * Round down for the bottom row, since we
				 * don't want to read beyond the lower border.
				 */
				v_resize_coeff = 8192 * ((c->in.rect.height >>
							 v_downsize_coeff) - 1)
						 / (c->out.rect.height - 1);
			} else {
				/*
				 * Round to closest for seams between tiles for
				 * minimal distortion.
				 */
				v_resize_coeff = DIV_ROUND_CLOSEST(8192 *
							(c->in.rect.height >>
							 v_downsize_coeff),
							c->out.rect.height);
			}

			/*
			 * With the scaling factor known, round up output height
			 * to IDMAC limitations
			 */
			c->out.rect.height = round_up(c->out.rect.height,
						      out_valign);

			/*
			 * Calculate input height from the last accessed input
			 * line given output height and scaling coefficients.
			 */
			c->in.rect.height = (DIV_ROUND_UP(
						(c->out.rect.height - 1) *
						v_resize_coeff, 8192) + 1)
					    << v_downsize_coeff;

			/* align height according to IDMAC restrictions */
			c->in.rect.height = round_up(c->in.rect.height,
				in_valign);

			memcpy(&c->out.pix, &out->pix,
			       sizeof(struct v4l2_pix_format));

			dev_dbg(ipu->dev,
				"%s: tile(%d,%d): %dx%d(%dx%d@%d,%d) -> %dx%d(%dx%d@%d,%d), resize: %dx%d\n",
				__func__, x, y,
				c->in.pix.width, c->in.pix.height,
				c->in.rect.width, c->in.rect.height,
				c->in.rect.left, c->in.rect.top,
				c->out.pix.width, c->out.pix.height,
				c->out.rect.width, c->out.rect.height,
				c->out.rect.left, c->out.rect.top,
				h_resize_coeff, v_resize_coeff);

			c->rsc = (v_downsize_coeff << 30) |
				 (v_resize_coeff << 16) |
				 (h_downsize_coeff << 14) |
				 h_resize_coeff;

			c++;
		}
	}

	*num_tiles = htiles * vtiles;

	return ctx;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_prepare);

int ipu_image_convert_run(struct ipu_soc *ipu, struct ipu_image *in,
			  struct ipu_image *out, struct image_convert_ctx *ctx,
			  int num_tiles, void (*complete)(void *ctx, int err),
			  void *complete_context, bool free_ctx)
{
	struct ipu_ic_priv *priv = ipu->ic_priv;
	struct ipu_ic *ic = &priv->task[IC_TASK_POST_PROCESSOR];
	unsigned long flags;
	int i;

	for (i = 0; i < num_tiles; i++) {
		ctx[i].in.phys0 = in->phys0;
		ctx[i].out.phys0 = out->phys0;
	}
	ctx[num_tiles - 1].complete = complete;
	ctx[num_tiles - 1].complete_context = complete_context;
	if (free_ctx)
		ctx[num_tiles - 1].freep = ctx;

	spin_lock_irqsave(&priv->lock, flags);

	for (i = 0; i < num_tiles; i++)
		list_add_tail(&ctx[i].list, &ic->image_list);

	queue_work(ic->workqueue, &ic->work);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_run);

static int ipu_image_convert_init(struct device *dev, struct ipu_soc *ipu,
		struct ipu_ic_priv *priv)
{
	int ret;
	struct ipu_ic *ic = ipu_ic_get(ipu, IC_TASK_POST_PROCESSOR);
	int irq = ipu_idmac_channel_irq(ipu, ic->output_channel,
					IPU_IRQ_EOF);

	ic->workqueue = create_singlethread_workqueue(dev_name(ipu->dev));
	if (!ic->workqueue)
		return -ENOMEM;

	INIT_WORK(&ic->work, ipu_image_convert_work);
	init_completion(&ic->complete);

	ret = devm_request_threaded_irq(dev, irq, NULL,
				ipu_image_convert_handler,
				IRQF_ONESHOT, "IC PP", ic);
	if (ret)
		goto err;

	return 0;
err:
	destroy_workqueue(ic->workqueue);
	return ret;
}

int ipu_ic_enable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 module = IPU_CONF_IC_EN;

	spin_lock_irqsave(&priv->lock, flags);

	if (ic->rotation)
		module |= IPU_CONF_ROT_EN;

	if (!priv->use_count)
		ipu_module_enable(priv->ipu, module);

	priv->use_count++;

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_ic_enable);

int ipu_ic_disable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 module = IPU_CONF_IC_EN | IPU_CONF_ROT_EN;

	spin_lock_irqsave(&priv->lock, flags);

	priv->use_count--;

	if (!priv->use_count)
		ipu_module_disable(priv->ipu, module);

	if (priv->use_count < 0)
		priv->use_count = 0;

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_ic_disable);

struct ipu_ic *ipu_ic_get(struct ipu_soc *ipu, enum ipu_ic_task task)
{
	struct ipu_ic_priv *priv = ipu->ic_priv;
	unsigned long flags;
	struct ipu_ic *ic, *ret;

	if (task >= IC_NUM_TASKS)
		return ERR_PTR(-EINVAL);

	ic = &priv->task[task];

	spin_lock_irqsave(&priv->lock, flags);

	if (ic->in_use) {
		ret = ERR_PTR(-EBUSY);
		goto unlock;
	}

	ic->in_use = true;
	ret = ic;

unlock:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_ic_get);

void ipu_ic_put(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	ic->in_use = false;
	spin_unlock_irqrestore(&priv->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_ic_put);

int ipu_ic_init(struct ipu_soc *ipu, struct device *dev,
		unsigned long base, unsigned long tpmem_base)
{
	struct ipu_ic_priv *priv;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ipu->ic_priv = priv;

	spin_lock_init(&priv->lock);
	priv->base = devm_ioremap(dev, base, PAGE_SIZE);
	if (!priv->base)
		return -ENOMEM;
	priv->tpmem_base = devm_ioremap(dev, tpmem_base, SZ_64K);
	if (!priv->tpmem_base)
		return -ENOMEM;

	dev_dbg(dev, "IC base: 0x%08lx remapped to %p\n", base, priv->base);

	priv->ipu = ipu;

	for (i = 0; i < IC_NUM_TASKS; i++) {
		INIT_LIST_HEAD(&priv->task[i].image_list);
		priv->task[i].task = i;
		priv->task[i].priv = priv;
		priv->task[i].reg = &ic_task_reg[i];
		priv->task[i].bit = &ic_task_bit[i];

		priv->task[i].input_channel = ipu_idmac_get(ipu,
							ic_task_ch[i].in);
		priv->task[i].output_channel = ipu_idmac_get(ipu,
							ic_task_ch[i].out);
		priv->task[i].rotation_input_channel = ipu_idmac_get(ipu,
							ic_task_ch[i].rot_in);
		priv->task[i].rotation_output_channel = ipu_idmac_get(ipu,
							ic_task_ch[i].rot_out);
		if (ic_task_ch[i].in_prev) {
			priv->task[i].input_channel_p = ipu_idmac_get(ipu,
							ic_task_ch[i].in_prev);
			priv->task[i].input_channel_n = ipu_idmac_get(ipu,
							ic_task_ch[i].in_next);
		}
	}

	ipu_image_convert_init(dev, ipu, priv);

	return 0;
}

void ipu_ic_exit(struct ipu_soc *ipu)
{
}

void ipu_ic_dump(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct ipu_soc *ipu = priv->ipu;

	dev_dbg(ipu->dev, "IC_CONF = \t0x%08X\n",
		ipu_ic_read(ic, IC_CONF));
	dev_dbg(ipu->dev, "IC_PRP_ENC_RSC = \t0x%08X\n",
		ipu_ic_read(ic, IC_PRP_ENC_RSC));
	dev_dbg(ipu->dev, "IC_PRP_VF_RSC = \t0x%08X\n",
		ipu_ic_read(ic, IC_PRP_VF_RSC));
	dev_dbg(ipu->dev, "IC_PP_RSC = \t0x%08X\n",
		ipu_ic_read(ic, IC_PP_RSC));
	dev_dbg(ipu->dev, "IC_CMBP_1 = \t0x%08X\n",
		ipu_ic_read(ic, IC_CMBP_1));
	dev_dbg(ipu->dev, "IC_CMBP_2 = \t0x%08X\n",
		ipu_ic_read(ic, IC_CMBP_2));
	dev_dbg(ipu->dev, "IC_IDMAC_1 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_1));
	dev_dbg(ipu->dev, "IC_IDMAC_2 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_2));
	dev_dbg(ipu->dev, "IC_IDMAC_3 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_3));
	dev_dbg(ipu->dev, "IC_IDMAC_4 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_4));
}
EXPORT_SYMBOL_GPL(ipu_ic_dump);
