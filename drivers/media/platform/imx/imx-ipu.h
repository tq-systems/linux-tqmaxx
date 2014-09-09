#ifndef __MEDIA_IMX_IPU_H
#define __MEDIA_IMX_IPU_H
#include <linux/videodev2.h>

struct ipu_fmt {
	u32 fourcc;
	const char *name;
	int bytes_per_pixel;
};

int ipu_enum_fmt(struct file *file, void *fh,
		struct v4l2_fmtdesc *f);
int ipu_enum_fmt_rgb(struct file *file, void *fh,
		struct v4l2_fmtdesc *f);
int ipu_enum_fmt_yuv(struct file *file, void *fh,
		struct v4l2_fmtdesc *f);
struct ipu_fmt *ipu_find_fmt_rgb(unsigned int pixelformat);
struct ipu_fmt *ipu_find_fmt_yuv(unsigned int pixelformat);
int ipu_try_fmt(struct file *file, void *fh,
		struct v4l2_format *f);
int ipu_try_fmt_rgb(struct file *file, void *fh,
		struct v4l2_format *f);
int ipu_try_fmt_yuv(struct file *file, void *fh,
		struct v4l2_format *f);
int ipu_s_fmt(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix);
int ipu_s_fmt_rgb(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix);
int ipu_s_fmt_yuv(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix);
int ipu_g_fmt(struct v4l2_format *f, struct v4l2_pix_format *pix);
int ipu_enum_framesizes(struct file *file, void *fh,
			struct v4l2_frmsizeenum *fsize);

#endif /* __MEDIA_IMX_IPU_H */
