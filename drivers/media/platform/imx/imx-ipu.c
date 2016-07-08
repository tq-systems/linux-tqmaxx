/*
 * i.MX IPUv3 common v4l2 support
 *
 * Copyright (C) 2011 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#include "imx-ipu.h"

static struct ipu_fmt ipu_fmt_yuv[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.name = "YUV 4:2:0 planar, YCbCr",
		.bytes_per_pixel = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU420,
		.name = "YUV 4:2:0 planar, YCrCb",
		.bytes_per_pixel = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.name = "YUV 4:2:2 planar, YCbCr",
		.bytes_per_pixel = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.name = "YUV 4:2:0 partial interleaved, YCbCr",
		.bytes_per_pixel = 1,
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,
		.name = "4:2:2, packed, UYVY",
		.bytes_per_pixel = 2,
	}, {
		.fourcc = V4L2_PIX_FMT_YUYV,
		.name = "4:2:2, packed, YUYV",
		.bytes_per_pixel = 2,
	},
};

static struct ipu_fmt ipu_fmt_rgb[] = {
	{
		.fourcc = V4L2_PIX_FMT_RGB32,
		.name = "RGB888",
		.bytes_per_pixel = 4,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB24,
		.name = "RGB24",
		.bytes_per_pixel = 3,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR24,
		.name = "BGR24",
		.bytes_per_pixel = 3,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB565,
		.name = "RGB565",
		.bytes_per_pixel = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_BGR32,
		.name = "BGR888",
		.bytes_per_pixel = 4,
	},
};

struct ipu_fmt *ipu_find_fmt_yuv(unsigned int pixelformat)
{
	struct ipu_fmt *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(ipu_fmt_yuv); i++) {
		fmt = &ipu_fmt_yuv[i];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(ipu_find_fmt_yuv);

struct ipu_fmt *ipu_find_fmt_rgb(unsigned int pixelformat)
{
	struct ipu_fmt *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(ipu_fmt_rgb); i++) {
		fmt = &ipu_fmt_rgb[i];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(ipu_find_fmt_rgb);

struct ipu_fmt *ipu_find_fmt(unsigned long pixelformat)
{
	struct ipu_fmt *fmt;

	fmt = ipu_find_fmt_yuv(pixelformat);
	if (fmt)
		return fmt;
	fmt = ipu_find_fmt_rgb(pixelformat);

	return fmt;
}
EXPORT_SYMBOL_GPL(ipu_find_fmt);

int ipu_try_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipu_fmt *fmt;

	v4l_bound_align_image(&f->fmt.pix.width, 8, 4096, 2,
			      &f->fmt.pix.height, 2, 4096, 1, 0);

	f->fmt.pix.field = V4L2_FIELD_NONE;

	fmt = ipu_find_fmt(f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	f->fmt.pix.bytesperline = f->fmt.pix.width * fmt->bytes_per_pixel;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
	if (fmt->fourcc == V4L2_PIX_FMT_YUV420 ||
	    fmt->fourcc == V4L2_PIX_FMT_YVU420 ||
	    fmt->fourcc == V4L2_PIX_FMT_NV12)
		f->fmt.pix.sizeimage = f->fmt.pix.sizeimage * 3 / 2;
	else if (fmt->fourcc == V4L2_PIX_FMT_YUV422P)
		f->fmt.pix.sizeimage *= 2;

	f->fmt.pix.priv = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_try_fmt);

int ipu_try_fmt_rgb(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipu_fmt *fmt;

	fmt = ipu_find_fmt_rgb(f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	return ipu_try_fmt(file, fh, f);
}
EXPORT_SYMBOL_GPL(ipu_try_fmt_rgb);

int ipu_try_fmt_yuv(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipu_fmt *fmt;

	fmt = ipu_find_fmt_yuv(f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	return ipu_try_fmt(file, fh, f);
}
EXPORT_SYMBOL_GPL(ipu_try_fmt_yuv);

int ipu_enum_fmt_rgb(struct file *file, void *fh,
		struct v4l2_fmtdesc *f)
{
	struct ipu_fmt *fmt;

	if (f->index >= ARRAY_SIZE(ipu_fmt_rgb))
		return -EINVAL;

	fmt = &ipu_fmt_rgb[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_enum_fmt_rgb);

int ipu_enum_fmt_yuv(struct file *file, void *fh,
		struct v4l2_fmtdesc *f)
{
	struct ipu_fmt *fmt;

	if (f->index >= ARRAY_SIZE(ipu_fmt_yuv))
		return -EINVAL;

	fmt = &ipu_fmt_yuv[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_enum_fmt_yuv);

int ipu_enum_fmt(struct file *file, void *fh,
		struct v4l2_fmtdesc *f)
{
	struct ipu_fmt *fmt;
	int index = f->index;

	if (index >= ARRAY_SIZE(ipu_fmt_yuv)) {
		index -= ARRAY_SIZE(ipu_fmt_yuv);
		if (index >= ARRAY_SIZE(ipu_fmt_rgb))
			return -EINVAL;
		fmt = &ipu_fmt_rgb[index];
	} else {
		fmt = &ipu_fmt_yuv[index];
	}

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_enum_fmt);

int ipu_s_fmt(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix)
{
	int ret;

	ret = ipu_try_fmt(file, fh, f);
	if (ret)
		return ret;

	pix->width = f->fmt.pix.width;
	pix->height = f->fmt.pix.height;
	pix->pixelformat = f->fmt.pix.pixelformat;
	pix->bytesperline = f->fmt.pix.bytesperline;
	pix->sizeimage = f->fmt.pix.sizeimage;
	pix->colorspace = f->fmt.pix.colorspace;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_s_fmt);

int ipu_s_fmt_rgb(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix)
{
	struct ipu_fmt *fmt;

	fmt = ipu_find_fmt_rgb(f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	return ipu_s_fmt(file, fh, f, pix);
}
EXPORT_SYMBOL_GPL(ipu_s_fmt_rgb);

int ipu_s_fmt_yuv(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix)
{
	struct ipu_fmt *fmt;

	fmt = ipu_find_fmt_yuv(f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	return ipu_s_fmt(file, fh, f, pix);
}
EXPORT_SYMBOL_GPL(ipu_s_fmt_yuv);

int ipu_g_fmt(struct v4l2_format *f, struct v4l2_pix_format *pix)
{
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat = pix->pixelformat;
	f->fmt.pix.bytesperline = pix->bytesperline;
	f->fmt.pix.width = pix->width;
	f->fmt.pix.height = pix->height;
	f->fmt.pix.sizeimage = pix->sizeimage;
	f->fmt.pix.colorspace = pix->colorspace;
	f->fmt.pix.priv = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_g_fmt);

int ipu_enum_framesizes(struct file *file, void *fh,
			struct v4l2_frmsizeenum *fsize)
{
	struct ipu_fmt *fmt;

	if (fsize->index != 0)
		return -EINVAL;

	fmt = ipu_find_fmt(fsize->pixel_format);
	if (!fmt)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = 1;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_width = 4096;
	fsize->stepwise.max_height = 4096;
	fsize->stepwise.step_width = fsize->stepwise.step_height = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_enum_framesizes);

MODULE_LICENSE("GPL");
