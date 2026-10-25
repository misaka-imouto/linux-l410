// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 (VCodec V500, VEDU V500R003) video encoder driver
 *
 * Implements a standard V4L2 mem2mem stateful video encoder for H.264 and HEVC.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/panic_notifier.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>

#include "hisi_venc.h"

static unsigned int clk_level = 1;
module_param(clk_level, uint, 0644);
MODULE_PARM_DESC(clk_level, "encoder clock: 0 = 640 MHz, 1 = 480 MHz, 2 = 332 MHz, 3 = 207.5 MHz");

/*
 * The encoder is still bring-up only: the VEDU needs a large register set and
 * internal work buffers (PMV, QPG, NBI, PME and the rec headers) that this
 * driver does not program yet.  Starting it faults in the SMMU and the
 * asynchronous SError panics the machine, so refuse to encode unless asked.
 */
static bool bringup;
module_param(bringup, bool, 0644);
MODULE_PARM_DESC(bringup, "allow starting the encoder (bring-up only, can panic the machine)");

/* bring-up: the EMUI10 register ABI may be shifted relative to our header */
static uint reg_shift;
module_param(reg_shift, uint, 0644);
MODULE_PARM_DESC(reg_shift, "byte offset added to VEDU core register writes");

static uint mode_extra;
module_param(mode_extra, uint, 0644);
MODULE_PARM_DESC(mode_extra, "bits ORed into the template MODE value");

/* bring-up bisect: skip the VEDU register programming / the start kick */
static bool skip_regs;
module_param(skip_regs, bool, 0644);
MODULE_PARM_DESC(skip_regs, "do not program the VEDU core registers");

static bool no_start;
module_param(no_start, bool, 0644);
MODULE_PARM_DESC(no_start, "program the VEDU but do not kick it");

/* ------------------------------------------------------------------ formats */

static const u32 hivenc_raw_fmts[] = {
	V4L2_PIX_FMT_NV12,
};

static const u32 hivenc_coded_fmts[] = {
	V4L2_PIX_FMT_H264,
	V4L2_PIX_FMT_HEVC,
};

static inline struct hivenc_ctx *file_to_ctx(struct file *file)
{
	return container_of(file->private_data, struct hivenc_ctx, fh);
}

static struct hivenc_q_data *hivenc_get_q_data(struct hivenc_ctx *ctx,
					       enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->out_q;
	return &ctx->cap_q;
}


/*
 * H.264 IDR register template for 320x240, extracted from the vendor userspace
 * HAL (libOMX.hisi.video.encoder.so, EMUI 10) by emulating
 * VencDrvInitParaCfgH264() + VeduHalCfgReg() with Unicorn.  The EMUI10
 * all_reg struct is our CFG layout shifted by +0x18, which is how the offsets
 * below were derived.  Address/stride/stream registers are not part of the
 * template; they are programmed per frame.
 */
static const struct {
	u16 reg;
	u32 val;
} hivenc_idr_template[] = {
	{0x0000, 0x000000cf},
	{0x0018, 0x40021504}, /* MODE, vedu_selfrst_en forced to 0 */
	{0x003c, 0x00008903},
	{0x0048, 0x00000001},
	{0x004c, 0x00080c00},
	{0x0050, 0x0001330a},
	{0x0058, 0x00000001},
	{0x005c, 0x00000016},
	{0x0060, 0x03030201},
	{0x0064, 0x05050404},
	{0x0068, 0x0f080808},
	{0x006c, 0x1919140f},
	{0x0070, 0x5555555a},
	{0x0074, 0x00000008},
	{0x0078, 0x20220080},
	{0x007c, 0x55555555},
	{0x0084, 0x05050303},
	{0x0088, 0x0f080808},
	{0x008c, 0x19191414},
	{0x0090, 0x02800000},
	{0x0094, 0x55555550},
	{0x0098, 0x14140101},
	{0x009c, 0x32322828},
	{0x00a0, 0xe6e6c8c8},
	{0x00a4, 0xffffffff},
	{0x00f0, 0x0000071f},
	{0x00f4, 0x00001806},
	{0x00f8, 0x00015858},
	{0x00fc, 0x55554505},
	{0x0100, 0x00052a05},
	{0x0104, 0x00000850},
	{0x0108, 0x00004858},
	{0x010c, 0x00100000},
	{0x0118, 0x0000f307},
	{0x011c, 0x00000001},
	{0x0128, 0x000f001f},
	{0x012c, 0x000f001f},
	{0x0140, 0xffffffff},
	{0x0144, 0x00000007},
	{0x0148, 0xffffffff},
	{0x014c, 0x00000007},
	{0x0150, 0xffffffff},
	{0x0154, 0x00000007},
	{0x0158, 0xffffffff},
	{0x015c, 0x00000007},
	{0x0160, 0x00000020},
	{0x0164, 0x00000033},
	{0x019c, 0x03c00040},
	{0x01a0, 0x03ac0040},
	{0x0254, 0x03ff03ff},
	{0x0258, 0x03ff03ff},
	{0x025c, 0x03ff03ff},
	{0x0260, 0x03ff03ff},
	{0x0264, 0x03ff03ff},
	{0x0268, 0x03ff03ff},
	{0x026c, 0x03ff03ff},
	{0x0270, 0x03ff03ff},
	{0x0274, 0x01ff01ff},
	{0x0278, 0x01ff01ff},
	{0x027c, 0x01ff01ff},
	{0x0280, 0x01ff01ff},
	{0x0284, 0x01ff01ff},
	{0x0288, 0x01ff01ff},
	{0x028c, 0x01ff01ff},
	{0x0290, 0x01ff01ff},
	{0x0298, 0x013f00ef},
	{0x02a4, 0x00000003},
	{0x02b0, 0x00000003},
	{0x02cc, 0x02010200},
	{0x0340, 0x007f3f3f},
	{0x038c, 0x00043300},
	{0x0390, 0x00003300},
	{0x0394, 0x00003300},
	{0x0398, 0x0003330a},
	{0x03a8, 0x0000001a},
	{0x03ac, 0x00055108},
	{0x03b0, 0x0c080401},
	{0x03b8, 0x0000a334},
	{0x03bc, 0x00008080},
	{0x03c0, 0x00000010},
	{0x03c4, 0x0030a330},
	{0x03c8, 0x02850000},
	{0x03d0, 0x003c0400},
	{0x03d4, 0x7d857b85},
	{0x03d8, 0x64280201},
	{0x03dc, 0x16b20040},
	{0x03e4, 0x00000100},
	{0x03f0, 0x000055ab},
	{0x03f4, 0x10100000},
	{0x03fc, 0x00008080},
	{0x0400, 0x00008080},
	{0x040c, 0x00010100},
	{0x0410, 0x00000100},
	{0x0418, 0x00000808},
	{0x0498, 0x00000f25},
	{0x04a4, 0x000001ff},
	{0x04a8, 0x05050f0f},
	{0x04ac, 0x87a0647f},
	{0x04b0, 0x009a1420},
	{0x04b4, 0x00010003},
	{0x04b8, 0x00400308},
	{0x04bc, 0x08030401},
	{0x04c0, 0x00000404},
	{0x04c4, 0xe68c8764},
	{0x04c8, 0x88788c74},
	{0x04cc, 0x014a010c},
	{0x04d0, 0x01040064},
	{0x04d4, 0x00f00010},
	{0x04d8, 0x00f00010},
	{0x04dc, 0x00000004},
	{0x0508, 0x0000000d},
	{0x067c, 0x00000002},
	{0x0684, 0x00000012},
	{0x068c, 0x00080008},
	{0x0690, 0x00080008},
	{0x069c, 0x00140100},
	{0x06a0, 0xa0c000ff},
	{0x06a4, 0x00000400},
	{0x06a8, 0x00c04010},
	{0x06ac, 0x00302010},
	{0x06b0, 0x00000124},
	{0x06b8, 0x0000050a},
	{0x08cc, 0x000001e0},
};

/* ------------------------------------------------------------------ power */

static int hivenc_power_on(struct hivenc_dev *venc)
{
	unsigned int level = min(clk_level, 3U);
	int ret;

	ret = regulator_enable(venc->media_supply);
	if (ret)
		return ret;

	ret = clk_prepare_enable(venc->clk);
	if (ret)
		goto err_media;

	ret = clk_set_rate(venc->clk, venc->clk_rate[3]);
	if (ret)
		goto err_clk;

	ret = regulator_enable(venc->venc_supply);
	if (ret)
		goto err_clk;

	if (venc->venc1_supply)
		regulator_enable(venc->venc1_supply);
	if (venc->clk1) {
		clk_prepare_enable(venc->clk1);
		if (level != 3)
			clk_set_rate(venc->clk1, venc->clk_rate[level]);
	}

	if (level != 3) {
		ret = clk_set_rate(venc->clk, venc->clk_rate[level]);
		if (ret)
			dev_warn(venc->dev, "clock rate %u failed: %d\n",
				 venc->clk_rate[level], ret);
	}

	venc->powered = true;
	hivenc_mmu_hw_setup(venc);

	/* Clear and unmask interrupts */
	vedu_write(venc, VEDU_VCPI_INTCLR, ~0U);
	vedu_write(venc, VEDU_VCPI_INTMASK,
		   RAWINT_VE_EOP | RAWINT_VEDU_SLC_END | RAWINT_VE_BUFFULL |
		   RAWINT_VEDU_TIMEOUT);

	/* is the block really clocked and decoding our writes? */
	vedu_write(venc, VEDU_REG_VEDU_VCPI_FRAMENO, 0x12345678);
	ret = vedu_read(venc, VEDU_REG_VEDU_VCPI_FRAMENO);
	dev_info(venc->dev, "VEDU probe: frameno readback %#x, clk %u Hz, clk1 %s\n",
		 ret, clk_get_rate(venc->clk), venc->clk1 ? "on" : "none");
	return 0;

err_clk:
	clk_disable_unprepare(venc->clk);
err_media:
	regulator_disable(venc->media_supply);
	return ret;
}

static void hivenc_power_off(struct hivenc_dev *venc)
{
	venc->powered = false;
	vedu_write(venc, VEDU_VCPI_INTMASK, 0);
	regulator_disable(venc->venc_supply);
	clk_set_rate(venc->clk, venc->clk_rate[3]);
	clk_disable_unprepare(venc->clk);
	regulator_disable(venc->media_supply);
	hivenc_mmu_powered_off(venc);
}

/* ------------------------------------------------------------------ watchdog */

static void hivenc_watchdog_func(struct work_struct *work)
{
	struct hivenc_dev *venc = container_of(to_delayed_work(work),
					       struct hivenc_dev, watchdog_work);
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct hivenc_ctx *ctx;

	mutex_lock(&venc->hw_lock);
	ctx = v4l2_m2m_get_curr_priv(venc->m2m_dev);
	if (!ctx) {
		mutex_unlock(&venc->hw_lock);
		return;
	}

	dev_err(venc->dev,
		"encoding timeout! rawint=%#x intstat=%#x timer=%#x busy=%#x strm=%u smmu=%#x rd=%#x wr=%#x\n",
		vedu_read(venc, VEDU_REG_FUNC_VCPI_RAWINT),
		vedu_read(venc, VEDU_REG_FUNC_VCPI_INTSTAT),
		vedu_read(venc, VEDU_REG_FUNC_VCPI_VEDU_TIMER),
		vedu_read(venc, VEDU_REG_FUNC_VCPI_BUS_IDLE_FLAG),
		vedu_read(venc, VEDU_REG_FUNC_VLC_PIC_STRMSIZE),
		vedu_read(venc, SMMU_INTSTAT_NS),
		vedu_read(venc, SMMU_ERR_RDADDR_NS),
		vedu_read(venc, SMMU_ERR_WRADDR_NS));

	/* Abort job */
	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (src_buf)
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
	if (dst_buf)
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);

	vedu_write(venc, VEDU_VCPI_INTCLR, ~0U);
	v4l2_m2m_job_finish(venc->m2m_dev, ctx->fh.m2m_ctx);
	mutex_unlock(&venc->hw_lock);
	pm_runtime_put_autosuspend(venc->dev);
}

/* ------------------------------------------------------------------ irq */

/* the SMMU latches its own fault interrupt and must be cleared separately */
static void hivenc_smmu_clear_fault(struct hivenc_dev *venc)
{
	u32 st = vedu_read(venc, SMMU_INTSTAT_NS);

	if (st)
		vedu_write(venc, SMMU_INTCLR_NS, st);
	vedu_write(venc, SMMU_MSTR_DEBUG(0x4c), 0x1f);
}

static irqreturn_t hivenc_irq(int irq, void *priv)
{
	struct hivenc_dev *venc = priv;
	u32 st, raw, smmu, mst, sid, addr, wsid, waddr;

	/* the vendor ISR reads FUNC_VCPI_INTSTAT, RAWINT is only for the info */
	st = vedu_read(venc, VEDU_REG_FUNC_VCPI_INTSTAT);
	raw = vedu_read(venc, VEDU_REG_FUNC_VCPI_RAWINT);
	smmu = vedu_read(venc, SMMU_INTSTAT_NS);
	mst = vedu_read(venc, SMMU_MSTR_DEBUG(0x40));
	{
		static bool mstr_dumped;

		if (!mstr_dumped && mst) {
			int i;

			mstr_dumped = true;
			for (i = 0; i < 24; i++)
				pr_info("hisi-venc mstr[%#04x] = %#x\n",
					SMMU_MSTR_BASE + i * 4,
					vedu_read(venc, SMMU_MSTR_BASE + i * 4));
		}
	}
	if (!st && !raw && !smmu && !mst) {
		dev_info_ratelimited(venc->dev,
			"irq: nothing latched (intstat %#x raw %#x smmu %#x mstr44 %#x)\n",
			st, raw, smmu, vedu_read(venc, SMMU_MSTR_DEBUG(0x44)));
		return IRQ_NONE;
	}

	sid = vedu_read(venc, SMMU_MSTR_DEBUG(0x50));
	addr = vedu_read(venc, SMMU_MSTR_DEBUG(0x54));
	wsid = vedu_read(venc, SMMU_MSTR_DEBUG(0x58));
	waddr = vedu_read(venc, SMMU_MSTR_DEBUG(0x5c));
	dev_info_ratelimited(venc->dev,
		"irq: intstat %#x raw %#x smmu %#x/%#x mstr %#x sid %#x addr %#x w %#x/%#x\n",
		st, raw, smmu, vedu_read(venc, SMMU_ERR_RDADDR_NS), mst,
		sid, addr, wsid, waddr);

	vedu_write(venc, VEDU_VCPI_INTCLR, st ? st : 0xffffffff);
	hivenc_smmu_clear_fault(venc);

	if (!st)
		return IRQ_HANDLED;

	venc->hw_status = st;
	return IRQ_WAKE_THREAD;
}

static irqreturn_t hivenc_irq_thread(int irq, void *priv)
{
	struct hivenc_dev *venc = priv;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct hivenc_ctx *ctx;
	u32 bytes_encoded = 0;
	bool is_idr;

	cancel_delayed_work(&venc->watchdog_work);

	mutex_lock(&venc->hw_lock);
	ctx = v4l2_m2m_get_curr_priv(venc->m2m_dev);
	if (!ctx) {
		mutex_unlock(&venc->hw_lock);
		pm_runtime_put_autosuspend(venc->dev);
		return IRQ_HANDLED;
	}

	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (!src_buf || !dst_buf) {
		mutex_unlock(&venc->hw_lock);
		pm_runtime_put_autosuspend(venc->dev);
		return IRQ_HANDLED;
	}

	if (venc->hw_status & RAWINT_VE_EOP) {
		u8 *dst_vaddr = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
		size_t hdr_len = 0;

		bytes_encoded = vedu_read(venc,
			ctx->proto == HIVENC_PROTO_HEVC ?
			VEDU_REG_FUNC_CABAC_PIC_STRMSIZE :
			VEDU_REG_FUNC_VLC_PIC_STRMSIZE);
		venc->encoded_bytes = bytes_encoded;

		is_idr = (ctx->frame_count == 0) || ctx->force_key_frame ||
			 ((ctx->gop_size > 0) && (ctx->frame_count % ctx->gop_size == 0));

		/* Prepend SPS/PPS or VPS/SPS/PPS for keyframes */
		if (is_idr && dst_vaddr) {
			size_t max_hdr_space = 512;
			u8 hdr_tmp[512];

			if (ctx->proto == HIVENC_PROTO_H264)
				hdr_len = hivenc_h264_write_headers(ctx, hdr_tmp, max_hdr_space);
			else
				hdr_len = hivenc_hevc_write_headers(ctx, hdr_tmp, max_hdr_space);

			if (hdr_len > 0 && bytes_encoded + hdr_len <= dst_buf->vb2_buf.planes[0].length) {
				memmove(dst_vaddr + hdr_len, dst_vaddr, bytes_encoded);
				memcpy(dst_vaddr, hdr_tmp, hdr_len);
				bytes_encoded += hdr_len;
			}
		}

		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, bytes_encoded);
		dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
		dst_buf->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
				    V4L2_BUF_FLAG_BFRAME);

		if (is_idr) {
			dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
			ctx->force_key_frame = false;
		} else {
			dst_buf->flags |= V4L2_BUF_FLAG_PFRAME;
		}

		/* Swap reconstructed buffer */
		ctx->cur_rec_idx ^= 1;
		ctx->has_ref = true;
		ctx->frame_count++;

		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
	} else {
		dev_err(venc->dev, "encode error interrupt status=%#x\n", venc->hw_status);
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
	}

	v4l2_m2m_job_finish(venc->m2m_dev, ctx->fh.m2m_ctx);
	mutex_unlock(&venc->hw_lock);
	pm_runtime_put_autosuspend(venc->dev);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ m2m */

static inline void hivenc_write64(struct hivenc_dev *venc, u32 off_l, u64 val)
{
	vedu_write(venc, off_l, lower_32_bits(val));
	vedu_write(venc, off_l + 4, upper_32_bits(val));
}

/* MODE: protocol in bits 3:2, frame type 11:10, idr 16, entropy 12 */
static u32 hivenc_vcpu_mode(struct hivenc_ctx *ctx, bool is_idr)
{
	u32 v = (1u << 2);		/* VEDU_H264 */
	u32 ft = is_idr ? 0 : 1;	/* I / P */

	if (ctx->proto == HIVENC_PROTO_HEVC)
		v = 0;			/* VEDU_H265 */
	v |= BIT(4);			/* register config mode (not cmdlist) */
	v |= BIT(5);			/* slice interrupt enable */
	v |= BIT(28);			/* vedu timer enable */
	v |= (ft & 3) << 10;
	if (is_idr)
		v |= BIT(16);
	if (ctx->proto == HIVENC_PROTO_HEVC)
		v |= BIT(6) | BIT(7);	/* sao luma/chroma */
	return v;
}

static void hivenc_device_run(void *priv)
{
	struct hivenc_ctx *ctx = priv;
	struct hivenc_dev *venc = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct hivenc_src_buffer *sbuf;
	struct hivenc_dst_buffer *dbuf;
	u32 width = ctx->out_q.width;
	u32 height = ctx->out_q.height;
	u32 y_stride = ctx->out_q.bytesperline;
	u32 rec_stride = ALIGN(width, 64);
	u32 rec_size = rec_stride * ALIGN(height, 16) * 3 / 2;
	u64 src_y_iova, src_c_iova, strm_iova;
	u64 rec_y_iova, rec_c_iova;
	u64 ref_y_iova = 0, ref_c_iova = 0;
	bool is_idr;
	int ret, rec_idx, ref_idx, i;

	mutex_lock(&venc->hw_lock);

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!src_buf || !dst_buf) {
		mutex_unlock(&venc->hw_lock);
		return;
	}
	sbuf = vb2_to_hivenc_src(&src_buf->vb2_buf);
	dbuf = vb2_to_hivenc_dst(&dst_buf->vb2_buf);

	ret = pm_runtime_resume_and_get(venc->dev);
	if (ret < 0) {
		v4l2_m2m_job_finish(venc->m2m_dev, ctx->fh.m2m_ctx);
		mutex_unlock(&venc->hw_lock);
		return;
	}

	if (skip_regs) {
		dev_warn(venc->dev, "skip_regs: powered on and SMMU programmed only\n");
		schedule_delayed_work(&venc->watchdog_work, msecs_to_jiffies(500));
		mutex_unlock(&venc->hw_lock);
		return;
	}

	rec_idx = ctx->cur_rec_idx;
	ref_idx = rec_idx ^ 1;
	if (!ctx->rec_buf[0].vaddr) {
		ret = hivenc_aux_alloc(venc, &ctx->rec_buf[0], rec_size);
		if (ret)
			goto err_pm;
		ret = hivenc_aux_alloc(venc, &ctx->rec_buf[1], rec_size);
		if (ret) {
			hivenc_aux_free(venc, &ctx->rec_buf[0]);
			goto err_pm;
		}
	}

	src_y_iova = sbuf->map.iova;
	src_c_iova = src_y_iova + y_stride * height;
	strm_iova = dbuf->map.iova;

	rec_y_iova = ctx->rec_buf[rec_idx].map.iova;
	rec_c_iova = rec_y_iova + rec_stride * ALIGN(height, 16);

	if (ctx->has_ref) {
		ref_y_iova = ctx->rec_buf[ref_idx].map.iova;
		ref_c_iova = ref_y_iova + rec_stride * ALIGN(height, 16);
	}

	is_idr = (ctx->frame_count == 0) || ctx->force_key_frame ||
		 ((ctx->gop_size > 0) && (ctx->frame_count % ctx->gop_size == 0));

	/* real VEDU V500 offsets, see hisi_venc_regs.h */
	vedu_write(venc, VEDU_REG_VEDU_VCPI_INTMASK, 0xcf);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_FRAMENO, ctx->frame_count);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_MODE, hivenc_vcpu_mode(ctx, is_idr));
	vedu_write(venc, VEDU_REG_VEDU_VCPI_PICSIZE_PIX,
		   ((width - 1) & 0xffff) | (((height - 1) & 0xffff) << 16));

	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_SRC_YADDR_L, src_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_SRC_CADDR_L, src_c_iova);
	/* STRIDE: chroma stride in the low half, luma stride in the high half */
	vedu_write(venc, VEDU_REG_VEDU_VCPI_STRIDE,
		   (y_stride & 0xffff) | ((y_stride & 0xffff) << 16));

	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_YADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_CADDR_L, rec_c_iova);
	/* no HFBC: point the rec headers at the rec buffer for now */
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_YH_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_CH_ADDR_L, rec_c_iova);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_REC_STRIDE,
		   (rec_stride & 0xffff) | ((rec_stride & 0xffff) << 16));

	if (!is_idr && ctx->has_ref) {
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFY_L0_ADDR_L, ref_y_iova);
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFC_L0_ADDR_L, ref_c_iova);
		vedu_write(venc, VEDU_REG_VEDU_VCPI_REF_L0_STRIDE,
			   (rec_stride & 0xffff) | ((rec_stride & 0xffff) << 16));
		vedu_write(venc, VEDU_REG_VEDU_VCPI_REF_FLAG, 1);
	} else {
		vedu_write(venc, VEDU_REG_VEDU_VCPI_REF_FLAG, 0);
	}

	vedu_write(venc, VEDU_REG_VEDU_PMV_POC_0, 0);
	vedu_write(venc, VEDU_REG_VEDU_PMV_POC_1, 0);

	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_STRMADDR_L, strm_iova);
	vedu_write(venc, VEDU_REG_VEDU_VLCST_STRMBUFLEN, ctx->cap_q.sizeimage);

	/* QPCFG: cr/cb offsets in 4:0 and 9:5, frame QP in 15:10 */
	vedu_write(venc, VEDU_REG_VEDU_VCPI_QPCFG, (ctx->qp & 0x3f) << 10);
	vedu_write(venc, VEDU_REG_VEDU_QPG_MAX_MIN_QP,
		   (10 & 0x3f) | ((51 & 0x3f) << 8) | BIT(16));
	vedu_write(venc, VEDU_REG_VEDU_VCPI_DBLKCFG, 0);

	/*
	 * No address register may stay at IOVA 0: the PME/NBI/QPG/REF masters
	 * all issue transactions as soon as the VEDU starts, and an access to
	 * IOVA 0 is what latches the SMMU master fault (0x1a040 = 0x1f) and
	 * stalls the pipeline.
	 */
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_LLILD_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_TUNLCELL_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_YH_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_CH_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFY_L0_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFC_L0_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFYH_L0_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFCH_L0_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFY_L1_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFC_L1_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFYH_L1_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REFCH_L1_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_PMELD_L0_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_PMELD_L1_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_PMEST_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_NBI_MVST_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_NBI_MVLD_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_PMEINFO_ST_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_PMEINFO_LD0_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_PMEINFO_LD1_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_QPGLD_INF_ADDR_L, rec_y_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_SKIPWEIGHT_LD_ADDR_L, rec_y_iova);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_PMEST_STRIDE, 0);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_PMELD_STRIDE, 0);

	/* ring-buffer pointers: both at the start of the stream buffer */
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_SWPTRADDR_L, strm_iova);
	hivenc_write64(venc, VEDU_REG_VEDU_VCPI_SRPTRADDR_L, strm_iova);

	/* linear 8-bit stream format, CAVLC, single slice */
	vedu_write(venc, VEDU_REG_VEDU_VCPI_STRFMT, 0);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_VLC_CONFIG, 0);
	vedu_write(venc, VEDU_REG_VEDU_VLC_SLCHDRPARA, 1);
	vedu_write(venc, VEDU_REG_VEDU_VLC_SLCHDRSTRM0, 0);
	vedu_write(venc, VEDU_REG_VEDU_VLC_SLCHDRSTRM1, 0);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_MULTISLC, 0);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_TIMEOUT, 0x00ffffff);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_INTCLR, 0xffffffff);

	if (reg_shift) {
		u32 sh = reg_shift;
		static const u32 internal[] = {
			VEDU_REG_VEDU_VCPI_LLILD_ADDR_L,
			VEDU_REG_VEDU_VCPI_TUNLCELL_ADDR_L,
			VEDU_REG_VEDU_VCPI_YH_ADDR_L,
			VEDU_REG_VEDU_VCPI_CH_ADDR_L,
			VEDU_REG_VEDU_VCPI_PMELD_L0_ADDR_L,
			VEDU_REG_VEDU_VCPI_PMEST_ADDR_L,
			VEDU_REG_VEDU_VCPI_NBI_MVST_ADDR_L,
			VEDU_REG_VEDU_VCPI_NBI_MVLD_ADDR_L,
			VEDU_REG_VEDU_VCPI_PMEINFO_ST_ADDR_L,
			VEDU_REG_VEDU_VCPI_QPGLD_INF_ADDR_L,
		};
		int i;

		vedu_write(venc, VEDU_REG_VEDU_VCPI_MODE + sh,
			   hivenc_vcpu_mode(ctx, is_idr));
		vedu_write(venc, VEDU_REG_VEDU_VCPI_PICSIZE_PIX + sh,
			   ((width - 1) & 0xffff) | (((height - 1) & 0xffff) << 16));
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_SRC_YADDR_L + sh, src_y_iova);
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_SRC_CADDR_L + sh, src_c_iova);
		vedu_write(venc, VEDU_REG_VEDU_VCPI_STRIDE + sh,
			   (y_stride & 0xffff) | ((y_stride & 0xffff) << 16));
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_YADDR_L + sh, rec_y_iova);
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_CADDR_L + sh, rec_c_iova);
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_YH_ADDR_L + sh, rec_y_iova);
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_REC_CH_ADDR_L + sh, rec_c_iova);
		vedu_write(venc, VEDU_REG_VEDU_VCPI_REC_STRIDE + sh,
			   (rec_stride & 0xffff) | ((rec_stride & 0xffff) << 16));
		hivenc_write64(venc, VEDU_REG_VEDU_VCPI_STRMADDR_L + sh, strm_iova);
		vedu_write(venc, VEDU_REG_VEDU_VCPI_QPCFG + sh, (ctx->qp & 0x3f) << 10);
		vedu_write(venc, VEDU_REG_VEDU_QPG_MAX_MIN_QP + sh,
			   (10 & 0x3f) | ((51 & 0x3f) << 8) | BIT(16));
		vedu_write(venc, VEDU_REG_VEDU_VCPI_STRFMT + sh, 0);
		vedu_write(venc, VEDU_REG_VEDU_VCPI_TIMEOUT + sh, 0x00ffffff);
		for (i = 0; i < (int)ARRAY_SIZE(internal); i++)
			hivenc_write64(venc, internal[i] + sh, rec_y_iova);
		dev_info(venc->dev, "reg_shift %#x applied\n", sh);
	}

	for (i = 0; i < ARRAY_SIZE(hivenc_idr_template); i++) {
		u32 val = hivenc_idr_template[i].val;

		if (hivenc_idr_template[i].reg == VEDU_REG_VEDU_VCPI_MODE)
			val |= mode_extra;
		vedu_write(venc, hivenc_idr_template[i].reg + reg_shift, val);
	}

	if (no_start) {
		dev_warn(venc->dev, "no_start: VEDU programmed but not started\n");
		schedule_delayed_work(&venc->watchdog_work, msecs_to_jiffies(500));
		mutex_unlock(&venc->hw_lock);
		return;
	}

	dev_info_ratelimited(venc->dev,
		"start: mode %#x size %ux%u src %#llx rec %#llx strm %#llx mstr40 %#x intstat %#x\n",
		vedu_read(venc, VEDU_REG_VEDU_VCPI_MODE), width, height,
		src_y_iova, rec_y_iova, strm_iova,
		vedu_read(venc, SMMU_MSTR_DEBUG(0x40)),
		vedu_read(venc, VEDU_REG_FUNC_VCPI_INTSTAT));
	schedule_delayed_work(&venc->watchdog_work, msecs_to_jiffies(500));
	vedu_write(venc, VEDU_REG_VEDU_VCPI_START + reg_shift, 0);
	vedu_write(venc, VEDU_REG_VEDU_VCPI_START + reg_shift, 1);
	mutex_unlock(&venc->hw_lock);
	return;

err_pm:
	pm_runtime_put_autosuspend(venc->dev);
	v4l2_m2m_job_finish(venc->m2m_dev, ctx->fh.m2m_ctx);
	mutex_unlock(&venc->hw_lock);
}

/* ------------------------------------------------------------------ vb2 */

static int hivenc_buf_init(struct vb2_buffer *vb)
{
	struct hivenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sg_table *sgt = vb2_dma_sg_plane_desc(vb, 0);
	struct hivenc_mapping *map;
	bool *mapped;
	int ret;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		map = &vb2_to_hivenc_src(vb)->map;
		mapped = &vb2_to_hivenc_src(vb)->mapped;
	} else {
		map = &vb2_to_hivenc_dst(vb)->map;
		mapped = &vb2_to_hivenc_dst(vb)->mapped;
	}
	ret = hivenc_mmu_map_sgt(ctx->dev, sgt, vb2_plane_size(vb, 0), map);
	if (ret)
		return ret;
	*mapped = true;
	return 0;
}

static void hivenc_buf_cleanup(struct vb2_buffer *vb)
{
	struct hivenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		struct hivenc_src_buffer *b = vb2_to_hivenc_src(vb);

		if (b->mapped)
			hivenc_mmu_unmap(ctx->dev, &b->map);
		b->mapped = false;
	} else {
		struct hivenc_dst_buffer *b = vb2_to_hivenc_dst(vb);

		if (b->mapped)
			hivenc_mmu_unmap(ctx->dev, &b->map);
		b->mapped = false;
	}
}

static int hivenc_queue_setup(struct vb2_queue *vq,
			      unsigned int *nbuffers, unsigned int *nplanes,
			      unsigned int sizes[], struct device *alloc_devs[])
{
	struct hivenc_ctx *ctx = vb2_get_drv_priv(vq);
	struct hivenc_q_data *q_data = hivenc_get_q_data(ctx, vq->type);

	if (*nplanes)
		return sizes[0] < q_data->sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = q_data->sizeimage;
	return 0;
}

static int hivenc_buf_prepare(struct vb2_buffer *vb)
{
	struct hivenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct hivenc_q_data *q_data = hivenc_get_q_data(ctx, vb->vb2_queue->type);

	if (vb2_plane_size(vb, 0) < q_data->sizeimage)
		return -EINVAL;

	return 0;
}

static void hivenc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct hivenc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int hivenc_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct hivenc_ctx *ctx = vb2_get_drv_priv(vq);

	if (!bringup) {
		dev_warn_once(ctx->dev->dev,
			      "hisi-venc is bring-up only, refusing to stream (bringup=1 to override)\n");
		return -EIO;
	}

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		ctx->frame_count = 0;
		ctx->has_ref = false;
	}
	return 0;
}

static void hivenc_stop_streaming(struct vb2_queue *vq)
{
	struct hivenc_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vbuf;

	while (true) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops hivenc_qops = {
	.queue_setup = hivenc_queue_setup,
	.buf_init = hivenc_buf_init,
	.buf_cleanup = hivenc_buf_cleanup,
	.buf_prepare = hivenc_buf_prepare,
	.buf_queue = hivenc_buf_queue,
	.start_streaming = hivenc_start_streaming,
	.stop_streaming = hivenc_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int hivenc_queue_init(void *priv, struct vb2_queue *src_vq,
			     struct vb2_queue *dst_vq)
{
	struct hivenc_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct hivenc_src_buffer);
	src_vq->ops = &hivenc_qops;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->vdev_lock;
	src_vq->dev = ctx->dev->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct hivenc_dst_buffer);
	dst_vq->ops = &hivenc_qops;
	dst_vq->mem_ops = &vb2_dma_sg_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->vdev_lock;
	dst_vq->dev = ctx->dev->dev;

	return vb2_queue_init(dst_vq);
}

/* ------------------------------------------------------------------ ioctls */

static int hivenc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, HIVENC_NAME, sizeof(cap->driver));
	strscpy(cap->card, "HiSilicon Kirin 990 VENC", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" HIVENC_NAME, sizeof(cap->bus_info));
	return 0;
}

static int hivenc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(hivenc_coded_fmts))
		return -EINVAL;

	f->pixelformat = hivenc_coded_fmts[f->index];
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int hivenc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(hivenc_raw_fmts))
		return -EINVAL;

	f->pixelformat = hivenc_raw_fmts[f->index];
	return 0;
}

static int hivenc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hivenc_ctx *ctx = file_to_ctx(file);
	struct hivenc_q_data *q_data = hivenc_get_q_data(ctx, f->type);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	pix->width = q_data->width;
	pix->height = q_data->height;
	pix->pixelformat = q_data->fourcc;
	pix->num_planes = 1;
	pix->plane_fmt[0].bytesperline = q_data->bytesperline;
	pix->plane_fmt[0].sizeimage = q_data->sizeimage;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	return 0;
}

static int hivenc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	u32 width = clamp_t(u32, pix->width, HIVENC_MIN_WIDTH, HIVENC_MAX_WIDTH);
	u32 height = clamp_t(u32, pix->height, HIVENC_MIN_HEIGHT, HIVENC_MAX_HEIGHT);

	width = ALIGN(width, 16);
	height = ALIGN(height, 16);

	pix->width = width;
	pix->height = height;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix->pixelformat = V4L2_PIX_FMT_NV12;
		pix->plane_fmt[0].bytesperline = ALIGN(width, 64);
		pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline * height * 3 / 2;
	} else {
		if (pix->pixelformat != V4L2_PIX_FMT_HEVC)
			pix->pixelformat = V4L2_PIX_FMT_H264;
		pix->plane_fmt[0].bytesperline = 0;
		pix->plane_fmt[0].sizeimage = clamp_t(u32, width * height,
						      512 * 1024, 4 * 1024 * 1024);
	}
	return 0;
}

static int hivenc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct hivenc_ctx *ctx = file_to_ctx(file);
	struct hivenc_q_data *q_data = hivenc_get_q_data(ctx, f->type);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = hivenc_try_fmt(file, priv, f);
	if (ret)
		return ret;

	q_data->fourcc = f->fmt.pix_mp.pixelformat;
	q_data->width = f->fmt.pix_mp.width;
	q_data->height = f->fmt.pix_mp.height;
	q_data->bytesperline = f->fmt.pix_mp.plane_fmt[0].bytesperline;
	q_data->sizeimage = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	if (!V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (q_data->fourcc == V4L2_PIX_FMT_HEVC)
			ctx->proto = HIVENC_PROTO_HEVC;
		else
			ctx->proto = HIVENC_PROTO_H264;
	}

	return 0;
}

static const struct v4l2_ioctl_ops hivenc_ioctl_ops = {
	.vidioc_querycap = hivenc_querycap,
	.vidioc_enum_fmt_vid_cap = hivenc_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out = hivenc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_cap_mplane = hivenc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = hivenc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = hivenc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = hivenc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = hivenc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = hivenc_s_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ------------------------------------------------------------------ controls */

static int hivenc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hivenc_ctx *ctx = container_of(ctrl->handler, struct hivenc_ctx,
					      ctrls.handler);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctx->force_key_frame = true;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctx->qp = ctrl->val;
		break;
	default:
		break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops hivenc_ctrl_ops = {
	.s_ctrl = hivenc_s_ctrl,
};

static int hivenc_ctrls_init(struct hivenc_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->ctrls.handler;

	v4l2_ctrl_handler_init(hdl, 8);

	ctx->ctrls.bitrate = v4l2_ctrl_new_std(hdl, &hivenc_ctrl_ops,
					       V4L2_CID_MPEG_VIDEO_BITRATE,
					       100000, 100000000, 10000, 4000000);
	ctx->ctrls.gop_size = v4l2_ctrl_new_std(hdl, &hivenc_ctrl_ops,
						V4L2_CID_MPEG_VIDEO_GOP_SIZE,
						1, 300, 1, 30);
	ctx->ctrls.force_key_frame = v4l2_ctrl_new_std(hdl, &hivenc_ctrl_ops,
						       V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
						       0, 0, 0, 0);
	ctx->ctrls.qp = v4l2_ctrl_new_std(hdl, &hivenc_ctrl_ops,
					  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
					  1, 51, 1, 26);
	ctx->ctrls.h264_profile = v4l2_ctrl_new_std_menu(hdl, &hivenc_ctrl_ops,
							 V4L2_CID_MPEG_VIDEO_H264_PROFILE,
							 V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
							 0, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
	ctx->ctrls.h264_level = v4l2_ctrl_new_std_menu(hdl, &hivenc_ctrl_ops,
						       V4L2_CID_MPEG_VIDEO_H264_LEVEL,
						       V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
						       0, V4L2_MPEG_VIDEO_H264_LEVEL_4_1);

	if (hdl->error) {
		int err = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return err;
	}

	ctx->fh.ctrl_handler = hdl;
	v4l2_ctrl_handler_setup(hdl);

	ctx->bitrate = 4000000;
	ctx->gop_size = 30;
	ctx->qp = 26;
	ctx->force_key_frame = false;
	return 0;
}

/* ------------------------------------------------------------------ file ops */

static int hivenc_open(struct file *file)
{
	struct hivenc_dev *venc = video_drvdata(file);
	struct hivenc_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = venc;

	/* Default queue settings */
	ctx->out_q.fourcc = V4L2_PIX_FMT_NV12;
	ctx->out_q.width = HIVENC_DEFAULT_WIDTH;
	ctx->out_q.height = HIVENC_DEFAULT_HEIGHT;
	ctx->out_q.bytesperline = ALIGN(ctx->out_q.width, 64);
	ctx->out_q.sizeimage = ctx->out_q.bytesperline * ctx->out_q.height * 3 / 2;

	ctx->cap_q.fourcc = V4L2_PIX_FMT_H264;
	ctx->cap_q.width = HIVENC_DEFAULT_WIDTH;
	ctx->cap_q.height = HIVENC_DEFAULT_HEIGHT;
	ctx->cap_q.bytesperline = 0;
	ctx->cap_q.sizeimage = 2 * 1024 * 1024;
	ctx->proto = HIVENC_PROTO_H264;

	ret = hivenc_ctrls_init(ctx);
	if (ret)
		goto err_free_ctx;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(venc->m2m_dev, ctx, hivenc_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_free_ctrls;
	}

	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_free_ctrls:
	v4l2_ctrl_handler_free(&ctx->ctrls.handler);
err_free_ctx:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int hivenc_release(struct file *file)
{
	struct hivenc_ctx *ctx = file->private_data ?
		container_of(file->private_data, struct hivenc_ctx, fh) : NULL;
	struct hivenc_dev *venc = video_drvdata(file);

	if (ctx) {
		v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
		v4l2_ctrl_handler_free(&ctx->ctrls.handler);
		v4l2_fh_del(&ctx->fh, file);
		v4l2_fh_exit(&ctx->fh);

		hivenc_aux_free(venc, &ctx->rec_buf[0]);
		hivenc_aux_free(venc, &ctx->rec_buf[1]);
		kfree(ctx);
	}
	return 0;
}

static const struct v4l2_file_operations hivenc_fops = {
	.owner = THIS_MODULE,
	.open = hivenc_open,
	.release = hivenc_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/*
 * SError from the VEDU/SMMU is asynchronous, so the faulting address and the
 * stream ID are only in the SMMU registers by the time the kernel panics.
 * Dump them so the next bring-up step has something to work with.
 */
static int hivenc_panic_notify(struct notifier_block *nb, unsigned long action,
			       void *data)
{
	struct hivenc_dev *venc = container_of(nb, struct hivenc_dev, panic_nb);

	if (!venc->powered)
		return NOTIFY_DONE;

	pr_emerg("hisi-venc: RAWINT %#x SMMU INTSTAT %#x RD %#x/%#x WR %#x/%#x\n",
		 vedu_read(venc, VEDU_REG_FUNC_VCPI_RAWINT),
		 vedu_read(venc, SMMU_INTSTAT_NS),
		 vedu_read(venc, SMMU_ERR_RDADDR_NS),
		 vedu_read(venc, SMMU_ERR_ADDR_MSB_NS),
		 vedu_read(venc, SMMU_ERR_WRADDR_NS),
		 vedu_read(venc, SMMU_ERR_ADDR_MSB_NS));
	pr_emerg("hisi-venc: MSTR SID/ADDR %#x %#x %#x %#x\n",
		 vedu_read(venc, SMMU_MSTR_DEBUG(0x44)),
		 vedu_read(venc, SMMU_MSTR_DEBUG(0x50)),
		 vedu_read(venc, SMMU_MSTR_DEBUG(0x54)),
		 vedu_read(venc, SMMU_MSTR_DEBUG(0x58)));
	return NOTIFY_DONE;
}

static const struct v4l2_m2m_ops hivenc_m2m_ops = {
	.device_run = hivenc_device_run,
};

/* ------------------------------------------------------------------ probe */

static int hivenc_runtime_resume(struct device *dev)
{
	return hivenc_power_on(dev_get_drvdata(dev));
}

static int hivenc_runtime_suspend(struct device *dev)
{
	hivenc_power_off(dev_get_drvdata(dev));
	return 0;
}

static const struct dev_pm_ops hivenc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(hivenc_runtime_suspend, hivenc_runtime_resume, NULL)
};

static int hivenc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hivenc_dev *venc;
	int ret;

	venc = devm_kzalloc(dev, sizeof(*venc), GFP_KERNEL);
	if (!venc)
		return -ENOMEM;

	platform_set_drvdata(pdev, venc);
	venc->dev = dev;
	mutex_init(&venc->vdev_lock);
	mutex_init(&venc->hw_lock);
	init_completion(&venc->job_done);
	INIT_DELAYED_WORK(&venc->watchdog_work, hivenc_watchdog_func);

	venc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(venc->regs))
		return PTR_ERR(venc->regs);

	venc->clk = devm_clk_get(dev, "clk_venc");
	if (IS_ERR(venc->clk))
		return dev_err_probe(dev, PTR_ERR(venc->clk), "Failed to get clk_venc\n");

	venc->media_supply = devm_regulator_get(dev, "ldo_media");
	if (IS_ERR(venc->media_supply))
		return dev_err_probe(dev, PTR_ERR(venc->media_supply), "Failed to get ldo_media\n");

	venc->venc_supply = devm_regulator_get(dev, "ldo_venc");
	if (IS_ERR(venc->venc_supply))
		return dev_err_probe(dev, PTR_ERR(venc->venc_supply), "Failed to get ldo_venc\n");

	venc->clk1 = devm_clk_get_optional(dev, "clk_venc1");
	if (IS_ERR(venc->clk1))
		venc->clk1 = NULL;
	venc->venc1_supply = devm_regulator_get_optional(dev, "ldo_venc1");
	if (IS_ERR(venc->venc1_supply))
		venc->venc1_supply = NULL;

	if (of_property_read_u32_array(dev->of_node, "enc_clk_rate",
				       venc->clk_rate, 4)) {
		venc->clk_rate[0] = 640000000;
		venc->clk_rate[1] = 480000000;
		venc->clk_rate[2] = 332000000;
		venc->clk_rate[3] = 207500000;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	venc->irq = platform_get_irq(pdev, 0);
	if (venc->irq < 0)
		return venc->irq;

	ret = devm_request_threaded_irq(dev, venc->irq, hivenc_irq, hivenc_irq_thread,
					IRQF_ONESHOT, dev_name(dev), venc);
	if (ret)
		return ret;

	ret = hivenc_mmu_init(venc);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	/* Initialize V4L2 and M2M devices */
	ret = v4l2_device_register(dev, &venc->v4l2_dev);
	if (ret)
		goto err_mmu;

	venc->m2m_dev = v4l2_m2m_init(&hivenc_m2m_ops);
	if (IS_ERR(venc->m2m_dev)) {
		ret = PTR_ERR(venc->m2m_dev);
		goto err_v4l2;
	}

	venc->vdev.lock = &venc->vdev_lock;
	venc->vdev.v4l2_dev = &venc->v4l2_dev;
	venc->vdev.fops = &hivenc_fops;
	venc->vdev.ioctl_ops = &hivenc_ioctl_ops;
	venc->vdev.release = video_device_release_empty;
	venc->vdev.vfl_dir = VFL_DIR_M2M;
	venc->vdev.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	strscpy(venc->vdev.name, HIVENC_NAME, sizeof(venc->vdev.name));
	video_set_drvdata(&venc->vdev, venc);

	ret = video_register_device(&venc->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	venc->panic_nb.notifier_call = hivenc_panic_notify;
	atomic_notifier_chain_register(&panic_notifier_list, &venc->panic_nb);

	dev_info(dev, "HiSilicon Kirin 990 VENC initialized (IRQ %d, clock %u Hz)\n",
		 venc->irq, venc->clk_rate[clk_level < 4 ? clk_level : 1]);
	return 0;

err_m2m:
	v4l2_m2m_release(venc->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&venc->v4l2_dev);
err_mmu:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	hivenc_mmu_fini(venc);
	return ret;
}

static void hivenc_remove(struct platform_device *pdev)
{
	struct hivenc_dev *venc = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&venc->watchdog_work);
	atomic_notifier_chain_unregister(&panic_notifier_list, &venc->panic_nb);
	video_unregister_device(&venc->vdev);
	v4l2_m2m_release(venc->m2m_dev);
	v4l2_device_unregister(&venc->v4l2_dev);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (venc->powered)
		hivenc_power_off(venc);
	hivenc_mmu_fini(venc);
}

static const struct of_device_id hivenc_of_match[] = {
	{ .compatible = "hisilicon,kirin990-venc" },
	{ .compatible = "hisilicon,HiVCodecV500-venc" },
	{ }
};
MODULE_DEVICE_TABLE(of, hivenc_of_match);

static struct platform_driver hivenc_driver = {
	.probe = hivenc_probe,
	.remove = hivenc_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = hivenc_of_match,
		.pm = &hivenc_pm_ops,
	},
};
module_platform_driver(hivenc_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 stateful video encoder (H.264/HEVC)");
MODULE_AUTHOR("misaka-imouto");
MODULE_LICENSE("GPL");
