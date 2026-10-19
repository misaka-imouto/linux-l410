/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon Kirin 990 (VCodec V500, VEDU V500R003) video encoder driver
 *
 * Header definitions for the V4L2 mem2mem video encoder.
 */

#ifndef HISI_VENC_H_
#define HISI_VENC_H_

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/genalloc.h>
#include <linux/io-pgtable.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>

#include "hisi_venc_hw.h"

#define HIVENC_NAME		"hisi-venc"

#define HIVENC_MIN_WIDTH	128
#define HIVENC_MIN_HEIGHT	128
#define HIVENC_MAX_WIDTH	4096
#define HIVENC_MAX_HEIGHT	2304

#define HIVENC_DEFAULT_WIDTH	1920
#define HIVENC_DEFAULT_HEIGHT	1080

struct hivenc_dev;
struct hivenc_ctx;

/* DMA contiguous auxiliary buffer (for internal reference and scratch frames) */
struct hivenc_aux_buf {
	void *vaddr;
	dma_addr_t dma;
	size_t size;
};

/* Buffer mapped into the encoder's private SMMU address space */
struct hivenc_mapping {
	u64 iova;
	size_t size;
};

/* Private SMMU state */
struct hivenc_mmu {
	struct mutex lock;
	struct io_pgtable_ops *ops;
	struct io_pgtable_cfg cfg;
	struct gen_pool *iova_pool;
	struct list_head deferred;
	struct hivenc_aux_buf err_rd;
	struct hivenc_aux_buf err_wr;
};

/* Queue configuration data (per direction: raw input or coded output) */
struct hivenc_q_data {
	u32 fourcc;
	u32 width;
	u32 height;
	u32 bytesperline;
	u32 sizeimage;
};

/* Controls state */
struct hivenc_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *bitrate;
	struct v4l2_ctrl *gop_size;
	struct v4l2_ctrl *h264_profile;
	struct v4l2_ctrl *h264_level;
	struct v4l2_ctrl *hevc_profile;
	struct v4l2_ctrl *hevc_level;
	struct v4l2_ctrl *force_key_frame;
	struct v4l2_ctrl *qp;
};

/* Encoding context per open instance */
struct hivenc_ctx {
	struct v4l2_fh fh;
	struct hivenc_dev *dev;
	struct hivenc_q_data out_q; /* Raw YUV input */
	struct hivenc_q_data cap_q; /* Compressed bitstream output */
	struct hivenc_ctrls ctrls;

	/* Internal reference and reconstruction buffers */
	struct hivenc_aux_buf rec_buf[2];
	struct hivenc_mapping rec_map[2];
	int cur_rec_idx;
	bool has_ref;

	u32 frame_count;
	u32 gop_size;
	u32 bitrate;
	u32 qp;
	bool force_key_frame;

	enum hivenc_protocol proto;
};

/* Device global state */
struct hivenc_dev {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device vdev;
	struct mutex vdev_lock;
	struct mutex hw_lock;

	void __iomem *regs;
	struct clk *clk;
	struct regulator *venc_supply;
	struct regulator *media_supply;
	u32 clk_rate[4];

	int irq;
	struct completion job_done;
	struct delayed_work watchdog_work;

	struct hivenc_mmu mmu;
	bool powered;
	u32 hw_status;
	u32 encoded_bytes;

	struct dentry *debugfs;
	u32 debug;
};

static inline u32 vedu_read(struct hivenc_dev *venc, u32 reg)
{
	return readl(venc->regs + reg);
}

static inline void vedu_write(struct hivenc_dev *venc, u32 reg, u32 val)
{
	writel(val, venc->regs + reg);
}

/* MMU interface */
int hivenc_mmu_init(struct hivenc_dev *venc);
void hivenc_mmu_fini(struct hivenc_dev *venc);
void hivenc_mmu_hw_setup(struct hivenc_dev *venc);
void hivenc_mmu_powered_off(struct hivenc_dev *venc);
int hivenc_mmu_map_sgt(struct hivenc_dev *venc, struct sg_table *sgt,
		       size_t size, struct hivenc_mapping *map);
void hivenc_mmu_unmap(struct hivenc_dev *venc, struct hivenc_mapping *map);

int hivenc_aux_alloc(struct hivenc_dev *venc, struct hivenc_aux_buf *buf,
		     size_t size);
void hivenc_aux_free(struct hivenc_dev *venc, struct hivenc_aux_buf *buf);

/* Codec helpers */
int hivenc_h264_write_headers(struct hivenc_ctx *ctx, u8 *buf, size_t max_len);
void hivenc_h264_setup_frame(struct hivenc_ctx *ctx, bool is_idr);

int hivenc_hevc_write_headers(struct hivenc_ctx *ctx, u8 *buf, size_t max_len);
void hivenc_hevc_setup_frame(struct hivenc_ctx *ctx, bool is_idr);

#endif /* HISI_VENC_H_ */
