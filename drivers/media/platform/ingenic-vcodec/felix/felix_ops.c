/*
*	Copyright (c) 2014 Ingenic Inc.
*	Author: qipengzhen <aric.pzqi@ingenic.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include <linux/slab.h>
#include <linux/crc32.h>

#include "felix_drv.h"
#include "felix_ops.h"

#undef pr_debug
#define pr_debug pr_info

#define fh_to_ctx(__fh) container_of(__fh, struct ingenic_vdec_ctx, fh)
#define NUM_SUPPORTED_FRAMESIZE		1

#define INGENIC_VDEC_MIN_W	160U
#define INGENIC_VDEC_MIN_H	120U
#define INGENIC_VDEC_MAX_W	2560U
#define INGENIC_VDEC_MAX_H	2048U
#define MAX_SUPPORT_FRAME_BUFFERS 16

static struct ingenic_video_fmt ingenic_video_formats[] = {
	/* out */
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.type = INGENIC_FMT_DEC,
		.num_planes = 1,
		.format = 0, //maybe unused?
	},

	/* cap */
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.type = INGENIC_FMT_FRAME,
		.num_planes = 2,
		.format = 0, //maybe unused??
	}
};

#define NUM_FORMATS	ARRAY_SIZE(ingenic_video_formats)
#define OUT_FMT_IDX	0
#define CAP_FMT_IDX 	ARRAY_SIZE(ingenic_video_formats) - 1

static const struct ingenic_vcodec_framesizes ingenic_vcodec_framesizes[] = {
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.stepwise = { INGENIC_VDEC_MIN_W, INGENIC_VDEC_MAX_W, 16,
			      INGENIC_VDEC_MIN_H, INGENIC_VDEC_MAX_H, 16},
	},
};


static int vidioc_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strlcpy(cap->driver, INGENIC_VCODEC_DEC_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "vpu-felix", sizeof(cap->bus_info));
	strlcpy(cap->card, "vpu-felix", sizeof(cap->card));

	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING |
                    V4L2_CAP_VIDEO_CAPTURE_MPLANE |
                    V4L2_CAP_VIDEO_OUTPUT_MPLANE |
		    V4L2_CAP_DEVICE_CAPS;

	return 0;
}


static int vidioc_enum_fmt(struct v4l2_fmtdesc *f, bool output_queue)
{
	struct ingenic_video_fmt *fmt;
	int i,j = 0;


	for(i = 0; i < NUM_FORMATS; i++) {
		if(output_queue && ingenic_video_formats[i].type != INGENIC_FMT_DEC)
			continue;
		if(!output_queue && ingenic_video_formats[i].type != INGENIC_FMT_FRAME)
			continue;

		if(j == f->index) {
			fmt = &ingenic_video_formats[i];
			f->pixelformat = fmt->fourcc;
			memset(f->reserved, 0, sizeof(f->reserved));
			return 0;
		}

		j++;
	}

	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap_mplane(struct file *file, void *priv,
					struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, false);
}
static int vidioc_enum_fmt_vid_out_mplane(struct file *file, void *priv,
					struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, true);
}

static int vidioc_try_fmt(struct v4l2_format *f, struct ingenic_video_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	int i;

	pix_fmt_mp->field = V4L2_FIELD_NONE;

	if(f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_fmt_mp->num_planes = 1;
		pix_fmt_mp->plane_fmt[0].bytesperline = 0;
	} else if(f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){
		int tmp_w, tmp_h;

		pix_fmt_mp->height = clamp(pix_fmt_mp->height,
					INGENIC_VDEC_MIN_H,
					INGENIC_VDEC_MAX_H);
		pix_fmt_mp->width = clamp(pix_fmt_mp->width,
					INGENIC_VDEC_MIN_W,
					INGENIC_VDEC_MAX_W);

		tmp_w = pix_fmt_mp->width;
		tmp_h = pix_fmt_mp->height;

		tmp_w = (tmp_w + 15) / 16 - 1;
		tmp_w = (tmp_w / 8) + 1;
		pix_fmt_mp->width = tmp_w * 128;

#if 0
		v4l_bound_align_image(&pix_fmt_mp->width,
					INGENIC_VDEC_MIN_W,
					INGENIC_VDEC_MAX_W, 8,
					&pix_fmt_mp->height,
					INGENIC_VDEC_MIN_H,
					INGENIC_VDEC_MAX_H, 0, 0);
#endif
		pr_debug("tmp_w %d tmp_h %d, w %d h %d\n",
				tmp_w, tmp_h,
				pix_fmt_mp->width,
				pix_fmt_mp->height);


		/* TODO: alignment handle ....*/
		pix_fmt_mp->num_planes = fmt->num_planes;
		pix_fmt_mp->plane_fmt[0].sizeimage =
				pix_fmt_mp->width * pix_fmt_mp->height;

		pix_fmt_mp->plane_fmt[0].bytesperline = pix_fmt_mp->width;

		if(pix_fmt_mp->num_planes == 2) {
			pix_fmt_mp->plane_fmt[1].sizeimage =
				pix_fmt_mp->width * pix_fmt_mp->height / 2;
			pix_fmt_mp->plane_fmt[2].sizeimage = 0;

			pix_fmt_mp->plane_fmt[1].bytesperline = pix_fmt_mp->width;
			pix_fmt_mp->plane_fmt[2].bytesperline = 0;
		} else if(pix_fmt_mp->num_planes == 3) {
			pix_fmt_mp->plane_fmt[1].sizeimage =
			pix_fmt_mp->plane_fmt[2].sizeimage =
				(pix_fmt_mp->width * pix_fmt_mp->height) / 4;

			pix_fmt_mp->plane_fmt[1].bytesperline =
			pix_fmt_mp->plane_fmt[2].bytesperline =
				pix_fmt_mp->width / 2;
		}
	}

	for(i = 0; i < pix_fmt_mp->num_planes; i++) {
		memset(&(pix_fmt_mp->plane_fmt[i].reserved[0]), 0x0,
				sizeof(pix_fmt_mp->plane_fmt[0].reserved));
	}

//	pix_fmt_mp->flags = 0;

	memset(&pix_fmt_mp->reserved, 0x0,
			sizeof(pix_fmt_mp->reserved));

	return 0;
}

static struct ingenic_vcodec_q_data *ingenic_vcodec_get_q_data(struct ingenic_vdec_ctx *ctx,
							enum v4l2_buf_type type)
{
	if(V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->q_data[INGENIC_Q_DATA_SRC];

	return &ctx->q_data[INGENIC_Q_DATA_DST];
}

static struct ingenic_video_fmt *ingenic_vcodec_find_format(struct v4l2_format *f)
{
	struct ingenic_video_fmt *fmt;
	int i;

	pr_debug("=========%s, %d, f->fmt.pix_mp.pixelformat %x\n", __func__, __LINE__, f->fmt.pix_mp.pixelformat);

	for(i = 0; i < NUM_FORMATS; i++) {
		fmt = &ingenic_video_formats[i];
		pr_debug("fmt->fourcc %x\n", fmt->fourcc);
		if(fmt->fourcc == f->fmt.pix_mp.pixelformat)
			return fmt;
	}

	pr_debug("=========Format not found ??????!\n");
	return NULL;
}

static int vidioc_s_fmt_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_vcodec_q_data *q_data;
	struct ingenic_video_fmt *fmt;
	int i, ret;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if(!vq) {
		pr_err("failed to get cap vq !\n");
		return -EINVAL;
	}

	if(vb2_is_busy(vq)) {
		pr_err("cap vq is busy!\n");
		return -EINVAL;
	}

	q_data = ingenic_vcodec_get_q_data(ctx, f->type);
	if(!q_data) {
		pr_err("fail to get cap q data!\n");
		return -EINVAL;
	}


	fmt = ingenic_vcodec_find_format(f);
	if(!fmt) {
		/* change to the first support cap format.*/
		f->fmt.pix.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		/* if buf type MPLANE, use pix_mp ..*/
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}


	q_data->fmt = fmt;

	ret = vidioc_try_fmt(f, q_data->fmt);
	if(ret)
		return ret;

	q_data->coded_width = f->fmt.pix_mp.width;
	q_data->coded_height = f->fmt.pix_mp.height;
	q_data->field = f->fmt.pix_mp.field;

	printk("-------s_fmt_cap: coded_width %d, coded_height %d\n", q_data->coded_width, q_data->coded_height);

	for(i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i] = plane_fmt->bytesperline;
		q_data->sizeimage[i] = plane_fmt->sizeimage;
	}


	return 0;
}

static int vidioc_s_fmt_out(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_vcodec_q_data *q_data;
	struct ingenic_video_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	int ret, i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if(!vq) {
		pr_err("fail to get out vq!\n");
		return -EINVAL;
	}
	if(vb2_is_busy(vq)) {
		pr_err("out vq is busy!\n");
		return -EINVAL;
	}

	q_data = ingenic_vcodec_get_q_data(ctx, f->type);
	if(!q_data) {
		pr_err("failed to get out q_data!\n");
		return -EINVAL;
	}


	pr_debug("s_fmt_out ... f->fmt %x %c%c%c%c\n", f->fmt.pix.pixelformat, f->fmt.pix.pixelformat);
	pr_debug("s_fmt_out ... f->fmt %x\n", f->fmt.pix_mp.pixelformat);
	fmt = ingenic_vcodec_find_format(f);
	if(!fmt) {
		/* change to the first support cap format.*/
		f->fmt.pix.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}


	pix_fmt_mp->height = clamp(pix_fmt_mp->height,
			INGENIC_VDEC_MIN_H,
			INGENIC_VDEC_MAX_H);
	pix_fmt_mp->width = clamp(pix_fmt_mp->width,
			INGENIC_VDEC_MIN_W,
			INGENIC_VDEC_MAX_W);

	q_data->visible_width = f->fmt.pix_mp.width;
	q_data->visible_height = f->fmt.pix_mp.height;
	q_data->fmt = fmt;
	ret = vidioc_try_fmt(f, q_data->fmt);
	if (ret)
		return ret;

	q_data->coded_width = f->fmt.pix_mp.width;
	q_data->coded_height = f->fmt.pix_mp.height;

	printk("--------s_fmt_out coded_width %d, coded_height %d\n", q_data->coded_width, q_data->coded_height);

	q_data->field = f->fmt.pix_mp.field;
	ctx->colorspace = f->fmt.pix_mp.colorspace;
	//ctx->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	//ctx->quantization = f->fmt.pix_mp.quantization;
	//ctx->xfer_func = f->fmt.pix_mp.xfer_func;


	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i] = plane_fmt->bytesperline;
		q_data->sizeimage[i] = plane_fmt->sizeimage;
	}


	return 0;
}

static int vidioc_g_fmt(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct ingenic_vcodec_q_data *q_data;
	int i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = ingenic_vcodec_get_q_data(ctx, f->type);

	pix->width = q_data->coded_width;
	pix->height = q_data->coded_height;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->field = q_data->field;
	pix->num_planes = q_data->fmt->num_planes;
	for (i = 0; i < pix->num_planes; i++) {
		pix->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		memset(&(pix->plane_fmt[i].reserved[0]), 0x0,
				sizeof(pix->plane_fmt[i].reserved));
	}

//	pix->flags = 0;
	pix->colorspace = ctx->colorspace;
//	pix->ycbcr_enc = ctx->ycbcr_enc;
//	pix->quantization = ctx->quantization;
//	pix->xfer_func = ctx->xfer_func;


	pr_debug("g_fmt f->type %d\n", f->type);

	pr_debug("pixel_format = %x\n", pix->pixelformat);
	pr_debug("pix->num_planes %d\n", pix->num_planes);

	return 0;
}

static int vidioc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct ingenic_video_fmt *fmt;
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	fmt = ingenic_vcodec_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[CAP_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}
	f->fmt.pix_mp.colorspace = ctx->colorspace;
	//f->fmt.pix_mp.ycbcr_enc = ctx->ycbcr_enc;
	//f->fmt.pix_mp.quantization = ctx->quantization;
	//f->fmt.pix_mp.xfer_func = ctx->xfer_func;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out_mplane(struct file *file, void *priv,
			struct v4l2_format *f)
{

	struct ingenic_video_fmt *fmt;

	pr_debug("==== %s, %d, fmt %x\n", __func__, __LINE__, f->fmt.pix.pixelformat);
	pr_debug("==== %s, %d,pix_mp %x\n", __func__, __LINE__, f->fmt.pix_mp.pixelformat);
	fmt = ingenic_vcodec_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		f->fmt.pix_mp.pixelformat = ingenic_video_formats[OUT_FMT_IDX].fourcc;
		fmt = ingenic_vcodec_find_format(f);
	}
	if (!f->fmt.pix_mp.colorspace) {
		f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
	//	f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	//	f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	//	f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	}

	pr_debug("==== %s, %d, fmt %x\n", __func__, __LINE__, f->fmt.pix.pixelformat);
	pr_debug("==== %s, %d,pix_mp %x\n", __func__, __LINE__, f->fmt.pix_mp.pixelformat);

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbufs)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	pr_debug("%s--%d\n", __func__, __LINE__);
	pr_debug("====== %s, %d, buf->type %d\n", __func__, __LINE__, reqbufs->type);
	pr_debug("============================ reqbufs: count: %d\n", reqbufs->count);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);

}

static int vidioc_querybuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);

}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	if(ctx->state == INGENIC_STATE_ABORT) {
		return -EIO;
	}
#if 0
	printk("---%s, %d, index: %d, type:%d [%s]\n",
		 __func__, __LINE__, buf->index, buf->type, buf->type == 10 ? "OUTPUT":"CAPTUR");
#endif
	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
#if 0
	printk("---%s, %d, index: %d, type:%d [%s]\n",
		 __func__, __LINE__, buf->index, buf->type, buf->type == 10 ? "OUTPUT":"CAPTUR");
#endif
	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_expbuf(struct file *file, void *priv,
	struct v4l2_exportbuffer *eb)
{

	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_expbuf(file, ctx->m2m_ctx, eb);
}

static int vidioc_create_bufs(struct file *file, void *priv,
			      struct v4l2_create_buffers *create)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_create_bufs(file, ctx->m2m_ctx, create);
}
static int vidioc_prepare_buf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	//TODO ....???

	return 0;
}

static int vidioc_streamon(struct file *file, void *priv,
			enum v4l2_buf_type type)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	pr_debug("====== %s, %d\n", __func__, __LINE__);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			enum v4l2_buf_type type)
{
	struct ingenic_vdec_ctx *ctx = fh_to_ctx(priv);
	pr_debug("====== %s, %d\n", __func__, __LINE__);


	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}


static int vidioc_g_selection(struct file *file, void *priv,
			struct v4l2_selection *s)
{
	// TODO

	return 0;
}

static int vidioc_s_selection(struct file *file, void *priv,
			struct v4l2_selection *s)
{
	//TODO
	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	int i = 0;
	if(fsize->index != 0)
		return -EINVAL;

	for(i = 0; i < NUM_SUPPORTED_FRAMESIZE; i++) {
		if(fsize->pixel_format != ingenic_vcodec_framesizes[i].fourcc)
			continue;

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise = ingenic_vcodec_framesizes[i].stepwise;
		return 0;
	}

	return -EINVAL;
}

const struct v4l2_ioctl_ops ingenic_vdec_ioctl_ops = {

	/* VIDIOC_QUERYCAP handler */
	.vidioc_querycap		= vidioc_querycap,

	/* VIDIOC_ENUM_FMT handlers */
	.vidioc_enum_fmt_vid_cap_mplane = vidioc_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out_mplane = vidioc_enum_fmt_vid_out_mplane,

	/* VIDIOC_G_FMT handlers */
	.vidioc_g_fmt_vid_cap_mplane	= vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= vidioc_g_fmt,

	/* VIDIOC_S_FMT handlers */
	.vidioc_s_fmt_vid_cap_mplane	= vidioc_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane	= vidioc_s_fmt_out,

	/* VIDIOC_TRY_FMT handlers */
	.vidioc_try_fmt_vid_cap_mplane	= vidioc_try_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_out_mplane	= vidioc_try_fmt_vid_out_mplane,

	/* Buffer handlers */
	.vidioc_reqbufs			= vidioc_reqbufs,
	.vidioc_querybuf		= vidioc_querybuf,
	.vidioc_qbuf			= vidioc_qbuf,
	.vidioc_dqbuf			= vidioc_dqbuf,
	.vidioc_expbuf			= vidioc_expbuf,

	.vidioc_create_bufs		= vidioc_create_bufs,
	.vidioc_prepare_buf		= vidioc_prepare_buf,

	/* Stream on/off */
	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,

	/* Crop ioctls */
	.vidioc_g_selection		= vidioc_g_selection,
	.vidioc_s_selection		= vidioc_s_selection,

	.vidioc_enum_framesizes		= vidioc_enum_framesizes,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

};


static void dump_vb2_buffer(struct vb2_buffer *vb2, const char *str)
{
	int i;
	printk("======dump vb2: %s=========\n", str);
	printk("vb2->num_planes:        %d\n", vb2->num_planes);
	printk("vb2->v4l2_buf.index:    %d\n", vb2->index);
	printk("vb2->v4l2_buf.type:     %d\n", vb2->type);
	//printk("vb2->v4l2_buf.sequence: %d\n", vb2->v4l2_buf.sequence);
	//printk("vb2->v4l2_buf.length:   %d\n", vb2->v4l2_buf.length);
	for(i = 0; i < vb2->num_planes; i++) {
		printk("planes@	%d : \n", i);
		printk("\tbyteused: 	%d\n", vb2->planes[i].bytesused);
		printk("\tlength: 	%d\n", vb2->planes[i].length);

		printk("\tvaddr: 0x%08x\n", vb2_plane_vaddr(vb2, i));
		printk("\tpaddr: 0x%08x\n", vb2_dma_contig_plane_dma_addr(vb2, i));

		/* data from vaddr to vaddr + length */
		print_hex_dump(KERN_INFO, "data@ ", DUMP_PREFIX_ADDRESS, 16, 1, vb2_plane_vaddr(vb2, i), 128, true);

		printk("----------------------------------------\n");
	}
}

static AVFrame frame;
#define av_frame_to_vcode_buf(f)	\
		container_of(f, struct ingenic_vcodec_buf, frame) \

static void m2mops_vdec_device_run(void *priv)
{
	struct ingenic_vdec_ctx *ctx = priv;
	struct ingenic_vdec_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	AVCodecContext *avctx = ctx->avctx;
	H264Context *h = ctx->h;
	AVFrame *f = NULL;
	AVPacket avpkt;
	int got_frame = 0;



	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);

	//dump_vb2_buffer(src_buf, "src_buf");
	//dump_vb2_buffer(dst_buf, "dst_buf");


	avpkt.data = vb2_plane_vaddr(src_buf, 0);
	avpkt.size = vb2_get_plane_payload(src_buf, 0);
	avpkt.data_pa = vb2_dma_contig_plane_dma_addr(src_buf, 0);

	/*如果获取到了目标buffer，就将该buffer给到vpu.*/
	if(dst_buf) {
		struct ingenic_vcodec_buf *buf = container_of(dst_buf,
				struct ingenic_vcodec_buf, vb);

		/*TODO: copy timestamp, display 顺序号可能不一样，这样dst_buf的时间戳可能会前后颠倒，这样会有问题吗？如何避免?*/
		dst_buf->timestamp = src_buf->timestamp;
		dst_buf->timecode = src_buf->timecode;
		dst_buf->sequence = src_buf->sequence;

		h264_enqueue_frame(h, &buf->frame);
	}

	h264_decode_frame(avctx, &frame, &got_frame, &avpkt);
	if(got_frame) {
		/*如果解码完一帧了，那就将所有的可显示的buffer都标记为buffer_done.*/
		/*drain display frame*/
		f = h264_dequeue_frame(h);
		if(f) {
			struct ingenic_vcodec_buf *done_buf = av_frame_to_vcode_buf(f);
			//printk("------done_buf: %x done frame f: %x-----buf->index:%d, f->index: %d\n", done_buf, f, done_buf->vb.v4l2_buf.index, f->index);


			vb2_set_plane_payload(&done_buf->vb, 0, f->buf[0]->size);
			vb2_set_plane_payload(&done_buf->vb, 1, f->buf[1]->size);
			v4l2_m2m_buf_done(&done_buf->vb, VB2_BUF_STATE_DONE);
		}
	}

	/*src_buf 只需要返回STATE_DONE.*/
	v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);


	v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->m2m_ctx);
}


static int m2mops_vdec_job_ready(void *m2m_priv)
{
	struct ingenic_vdec_ctx *ctx = m2m_priv;

	if(ctx->state == INGENIC_STATE_ABORT || ctx->state == INGENIC_STATE_IDLE) {
		pr_info("job not ready, ctx->state %d\n", ctx->state);
		return 0;
	}

	return 1;
}


static void m2mops_vdec_job_abort(void *priv)
{
	struct ingenic_vdec_ctx *ctx = priv;
	ctx->state = INGENIC_STATE_ABORT;
}


static void m2mops_vdec_lock(void *m2m_priv)
{
	struct ingenic_vdec_ctx *ctx = m2m_priv;

	//TODO
	mutex_lock(&ctx->dev->dev_mutex);
}

static void m2mops_vdec_unlock(void *m2m_priv)
{
	struct ingenic_vdec_ctx *ctx = m2m_priv;

	mutex_unlock(&ctx->dev->dev_mutex);
}

const struct v4l2_m2m_ops ingenic_vdec_m2m_ops = {
	.device_run		= m2mops_vdec_device_run,
	.job_ready		= m2mops_vdec_job_ready,
	.job_abort		= m2mops_vdec_job_abort,
	.lock			= m2mops_vdec_lock,
	.unlock			= m2mops_vdec_unlock,
};

static int vb2ops_vcodec_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				void *alloc_ctxs[])
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct ingenic_vcodec_q_data *q_data;
	int i;

	pr_debug("%s--%d\n", __func__, __LINE__);

	q_data = ingenic_vcodec_get_q_data(ctx, vq->type);
	if(q_data == NULL)
		return -EINVAL;

	pr_debug("%s--%d, vq->type  %d\n", __func__, __LINE__, vq->type);

	if(*nplanes) {
		for(i = 0; i < *nplanes; i++) {
			if(sizes[i] < q_data->sizeimage[i]) {
				pr_debug("---sizes  %d %d, %d\n", *nplanes, sizes[i], q_data->sizeimage[i]);
				return -EINVAL;
			}
		}
	} else {
		*nplanes = q_data->fmt->num_planes;
		for (i = 0; i < *nplanes; i++) {
			sizes[i] = q_data->sizeimage[i];
			alloc_ctxs[i] = ctx->dev->alloc_ctx;
		}
	}


	pr_debug("*nplanes %d, sizes[0] %d\n", *nplanes, sizes[0]);
	for(i = 0; i<*nplanes ; i++) {
		pr_debug("plane %d, sizes[%d] %d, type  %d\n", i, i, sizes[i], vq->type);
	}

	if(*nbuffers > MAX_SUPPORT_FRAME_BUFFERS)
		*nbuffers = MAX_SUPPORT_FRAME_BUFFERS;

	pr_debug("%s--%d\n", __func__, __LINE__);
	return 0;
}

static int vb2ops_vcodec_buf_prepare(struct vb2_buffer *vb)
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ingenic_vcodec_q_data *q_data;
	int i;

#if 0
	printk("---%s, %d, index: %d, type:%d\n",
		 __func__, __LINE__, vb->v4l2_buf.index, vb->vb2_queue->type);
#endif
	q_data = ingenic_vcodec_get_q_data(ctx, vb->vb2_queue->type);

	for(i = 0; i < q_data->fmt->num_planes; i++) {
		if(vb2_plane_size(vb, i) > q_data->sizeimage[i]) {

			pr_err("Failed to prepare buf!\n");

			return -EINVAL;
		}
	}

#if 0
	pr_debug("%s--%d\n", __func__, __LINE__);
#endif
	return 0;
}


static int vb2ops_vcodec_buf_init(struct vb2_buffer *vb)
{
	struct ingenic_vcodec_buf *buf = container_of(vb,
                                struct ingenic_vcodec_buf, vb);

	int i;
	AVFrame *f = &buf->frame;

	f->index = vb->index;
	for(i = 0; i < vb->num_planes; i++) {
		f->buf[i] = av_buffer_create(vb2_plane_vaddr(vb, i),
					vb2_dma_contig_plane_dma_addr(vb, i),
					vb2_plane_size(vb, i)
					);
	}
	return 0;
}
static int vb2ops_vcodec_buf_cleanup(struct vb2_buffer *vb)
{
	struct ingenic_vcodec_buf *buf = container_of(vb,
                                struct ingenic_vcodec_buf, vb);
	int i;
	AVFrame *f = &buf->frame;

	for(i = 0; i < vb->num_planes; i++) {
		av_buffer_del(f->buf[i]);
	}

	return 0;
}

static void vb2ops_vcodec_buf_queue(struct vb2_buffer *vb)
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ingenic_vcodec_buf *buf = container_of(vb,
                                struct ingenic_vcodec_buf, vb);
	H264Context *h = ctx->h;

	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
	return;
}

static int vb2ops_vcodec_buf_finish(struct vb2_buffer *vb)
{
	/* call back before dqbuf to user. */
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	return 0;
}


static int vb2ops_vcodec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(q);
//	struct venc_sw_param *sw = ctx->sw;
	struct ingenic_vcodec_q_data *q_data_src;
	struct ingenic_vcodec_q_data *q_data_dst;
	int ret = 0;

	pr_debug("%s--%d\n", __func__, __LINE__);

	pr_debug("%s--%d, type: %d, count: %d", __func__, __LINE__, q->type, count);

	if(V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->output_stopped = 0;
	} else {
		ctx->capture_stopped = 0;
	}

	pr_debug("ctx->output_stopped: %d, ctx->capture_stopped: %d\n", ctx->output_stopped, ctx->capture_stopped);

	if(ctx->output_stopped  || ctx->capture_stopped)
		return 0;

	/*　如果两个queue都start了，就可以初始化workbuf了，这里主要根据输入的格式，输出的格式，申请除了用户空间q 和 dq之外的自己使用的buffer.*/

	q_data_src = ingenic_vcodec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	q_data_dst = ingenic_vcodec_get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

#if 0
	sw->width = q_data_src->visible_width;
	sw->height = q_data_src->visible_height;
	sw->format = q_data_src->fmt->format;
	sw->vdma_chain_len = 40960 + 256;

	sw->src_crc = 0;

	ret = helix_vcodec_alloc_workbuf(ctx);
	if(ret < 0) {
		pr_err("Falied to alloc vpu workbuf!\n");
		return ret;
	}



//	ctx->state = INGENIC_STATE_HEADER;

	/* do sps/pps encoder and stored it in ctx.*/
	helix_vcodec_h264_encode_headers(ctx);
	/* State -> started?? */

	if(sw->i_cabac) {
		h264_cabac_init();
	} else {
		printk("Only support cabac.!\n");
	}
#endif
	ctx->state = INGENIC_STATE_RUNNING;
	pr_debug("%s--%d\n", __func__, __LINE__);
	return 0;
}

static int vb2ops_vcodec_stop_streaming(struct vb2_queue *q)
{

	struct ingenic_vdec_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	int ret = 0;
	pr_debug("%s--%d\n", __func__, __LINE__);

//	printk("------ sw->src_crc %x\n", ctx->sw->src_crc);
	/* stop both output and capture .*/

	/*　当上层输入的数据没有了，就会draining, 只会停止OUTPUT stream,
		但是，此时，上层还是会进行dequeue CAPTURE stream，
		而此时，CAPTURE stream又没有数据，肯定会失败。

		所以应该上层判断是否没有原始数据了，如果没有原始数据，就不要再polling了。

	上层实现有问题了??????

	*/

//	if(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {

	/* KERNEL 3.10 .*/
		while((dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
			dst_buf->vb2_buf.planes[0].bytesused = 0;
			dst_buf->flags |= V4L2_BUF_FLAG_DONE;
			do_gettimeofday(&dst_buf->timestamp);
			/*保证两个的时间戳不一样.*/
			dst_buf->timestamp = ns_to_timeval((timeval_to_ns(&dst_buf->timestamp) + 10));
			/* work around, return buffer to userspace with 0 byteused. */
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		}
//	} else {
		while((src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx))) {
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		}
//	}

	if(q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ctx->capture_stopped = 1;
	else
		ctx->output_stopped = 1;

	if((ctx->capture_stopped && ctx->output_stopped) == 0) {
		return ret;
	}

#if 0
	/* State -> stopped ?? */

	helix_vcodec_free_workbuf(ctx);

	ctx->state = INGENIC_STATE_IDLE;
#endif

	pr_debug("%s--%d\n", __func__, __LINE__);
	return ret;
}

static const struct vb2_ops ingenic_vcodec_vb2_ops = {
	.queue_setup		= vb2ops_vcodec_queue_setup,
	.buf_prepare		= vb2ops_vcodec_buf_prepare,
	.buf_init		= vb2ops_vcodec_buf_init,
	.buf_cleanup		= vb2ops_vcodec_buf_cleanup,
	.buf_queue		= vb2ops_vcodec_buf_queue,
	.buf_finish		= vb2ops_vcodec_buf_finish,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= vb2ops_vcodec_start_streaming,
	.stop_streaming		= vb2ops_vcodec_stop_streaming,
};

int ingenic_vcodec_vdec_queue_init(void *priv, struct vb2_queue *src_vq,
	struct vb2_queue *dst_vq)
{
	struct ingenic_vdec_ctx *ctx = priv;
	int ret = 0;

	/* TODO: to support VB2_USERPTR with dmmu?. */
	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes	= VB2_DMABUF | VB2_MMAP;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size	= sizeof(struct ingenic_vcodec_buf);
	src_vq->ops		= &ingenic_vcodec_vb2_ops;
	src_vq->mem_ops		= &vb2_dma_contig_memops;
	src_vq->lock		= &ctx->dev->dev_mutex;
	src_vq->timestamp_flags  = V4L2_BUF_FLAG_TIMESTAMP_COPY;
//	src_vq->dev		= ctx->dev->dev;

	ret = vb2_queue_init(src_vq);
	if(ret)
		return ret;


	dst_vq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes	= VB2_DMABUF | VB2_MMAP;
	dst_vq->drv_priv	= ctx;
	dst_vq->buf_struct_size	= sizeof(struct ingenic_vcodec_buf);
	dst_vq->ops		= &ingenic_vcodec_vb2_ops;
	dst_vq->mem_ops		= &vb2_dma_contig_memops;
	dst_vq->lock		= &ctx->dev->dev_mutex;
	dst_vq->timestamp_flags  = V4L2_BUF_FLAG_TIMESTAMP_COPY;
//	dst_vq->dev		= ctx->dev->dev;

	ret = vb2_queue_init(dst_vq);
	if(ret) {
		vb2_queue_release(src_vq);
	}

	return ret;
}


int ingenic_vcodec_init_default_params(struct ingenic_vdec_ctx *ctx)
{
	int ret = 0;
	ctx->capture_stopped = 1;
	ctx->output_stopped = 1;
	ctx->state = INGENIC_STATE_IDLE;

	return ret;
}

int ingenic_vcodec_deinit_default_params(struct ingenic_vdec_ctx *ctx)
{

}
