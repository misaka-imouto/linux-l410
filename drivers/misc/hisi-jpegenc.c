// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 (ISP/VCodec) hardware JPEG encoder - bring-up.
 *
 * The JPGENC block is at 0xE8000000, behind the ISP subsystem, with its write
 * DMA going through the CVDR at 0xE8006000 and a private SMMU at 0xE8020000.
 * This driver is a minimal, test-only implementation: power on, load the
 * vendor Huffman/quantisation tables, point the input at a DMA-coherent NV12
 * frame, point the CVDR at an output buffer and start one 320x240 encode.
 * It is deliberately not a V4L2 driver yet.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>

#include "hisi-jpegenc-tables.h"

/* JPGENC */
#define JPGENC_ENCODE		0x04
#define JPGENC_INIT		0x08
#define JPGENC_HRIGHT1		0x18
#define JPGENC_VBOTTOM		0x1c
#define JPGENC_PIC_FORMAT	0x20
#define JPGENC_RESTART		0x24
#define JPGENC_TQ_Y		0x28
#define JPGENC_TQ_U		0x2c
#define JPGENC_TQ_V		0x30
#define JPGENC_TABLE_DATA	0x3c
#define JPGENC_TABLE_ID		0x40
#define JPGENC_TAC0_LEN		0x44
#define JPGENC_TDC0_LEN		0x48
#define JPGENC_TAC1_LEN		0x4c
#define JPGENC_TDC1_LEN		0x50
#define JPGENC_ERROR_RIS	0x6c
#define JPGENC_STATUS_RIS	0x80
#define JPGENC_STATUS_ICR	0x88
#define JPGENC_ERROR_ICR	0x74
#define JPGENC_Y_ADDR		0x94
#define JPGENC_UV_ADDR		0x98
#define JPGENC_STRIDE		0x9c
#define JPGENC_SYNCCFG		0x100
#define JPGENC_HRIGHT2		0x104
#define JPGENC_BYTE_CNT		0x108
#define JPGENC_PREFETCH		0x10c
#define JPGENC_PREREAD		0x120
#define JPGENC_INPUT_SWAP	0x124
#define JPGENC_FORCE_CLK_ON	0x130

/*
 * CVDR write port 25 (huawei,cvdr = <type, rd-port, wr-port> = <0 4 0x19>).
 * The "update" register set is the one the vendor uses when
 * huawei,wr_port_addr_update is non-zero.
 */
#define CVDR_VP_WR_CFG_25	0x1c0
#define CVDR_VP_WR_AXI_FS_25	0x1c4
#define CVDR_VP_WR_AXI_LINE_25	0x1c8
#define CVDR_VP_WR_IF_CFG_25	0x1cc
#define CVDR_LIMITER_25		0x894
#define CVDR_NR_RD_CFG_4	0x1570
#define CVDR_NR_RD_LIMIT_4	0x1578

/* SMMU */
#define SMMU_GLOBAL_BYPASS	0x00
#define SMMU_SMRX_NS(n)		(0x20 + 4 * (n))

#define PIX_FMT_D64		0x1E
#define JPGENC_HEAD_SIZE	640
#define JPGENC_OUT_SIZE		(512 * 1024)

struct hisi_jpegenc {
	struct device *dev;
	void __iomem *regs;
	void __iomem *subctrl;
	void __iomem *cvdr;
	void __iomem *smmu;

	struct regulator *media_supply;
	struct regulator *srt_supply;
	struct clk *clk_func;
	struct clk *aclk;
	struct clk *pclk;

	bool powered;

	u8 *out;
	dma_addr_t out_dma;
	size_t out_size;
	u32 byte_cnt;
	bool have_image;
	struct dentry *dbg;
};

static bool do_probe;
module_param(do_probe, bool, 0644);
MODULE_PARM_DESC(do_probe, "power the JPEG block at probe (0 = map resources only)");

static bool test;
module_param(test, bool, 0644);
MODULE_PARM_DESC(test, "run one 320x240 NV12 encode at probe");
static bool flat;
module_param(flat, bool, 0644);
MODULE_PARM_DESC(flat, "fill the test frame with constant 128 instead of a gradient");
static uint sync_cfg = 0x5;
module_param(sync_cfg, uint, 0644);
MODULE_PARM_DESC(sync_cfg, "JPGENC_SYNCCFG value (source bit0, sram_noopt bit2)");
static uint jpeg_config;
module_param(jpeg_config, uint, 0644);
MODULE_PARM_DESC(jpeg_config, "JPGENC_JPE_CONFIG_REG value");
static bool seq;
module_param(seq, bool, 0644);
MODULE_PARM_DESC(seq, "run a G,G,F,F,G,G sequence in encode_test");
static uint sweep;
module_param(sweep, uint, 0644);
MODULE_PARM_DESC(sweep, "0=off, 1=single-bit, 2=pairwise, 4=16-bit values (or them)");
static uint sweep_max = 0xff;
module_param(sweep_max, uint, 0644);
MODULE_PARM_DESC(sweep_max, "last value for sweep=4 (default 0xff)");
static uint sweep_reg = 0xffffffff;
module_param(sweep_reg, uint, 0644);
MODULE_PARM_DESC(sweep_reg, "limit the value sweep to one register offset");
static uint cvdr_start;
module_param(cvdr_start, uint, 0644);
MODULE_PARM_DESC(cvdr_start, "first CVDR offset for sweep=8");
static uint cvdr_end = 0x200;
module_param(cvdr_end, uint, 0644);
MODULE_PARM_DESC(cvdr_end, "last CVDR offset for sweep=8");

static const struct of_device_id hisi_jpegenc_of_match[] = {
	{ .compatible = "huawei,hjpeg" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_jpegenc_of_match);

static void jw(struct hisi_jpegenc *j, u32 off, u32 val)
{
	writel(val, j->regs + off);
}

static u32 jr(struct hisi_jpegenc *j, u32 off)
{
	return readl(j->regs + off);
}

static void table_data(struct hisi_jpegenc *j, const u8 *data, int n)
{
	int i;

	for (i = 0; i + 1 < n; i += 2)
		jw(j, JPGENC_TABLE_DATA, (data[i] << 8) | data[i + 1]);
}

static void table_write(struct hisi_jpegenc *j, u8 id, const u8 *data, int n)
{
	jw(j, JPGENC_TABLE_ID, id);
	table_data(j, data, n);
}

static void huff_init(struct hisi_jpegenc *j)
{
	jw(j, JPGENC_TDC0_LEN, ARRAY_SIZE(luma_dc_bits) + ARRAY_SIZE(luma_dc_value));
	jw(j, JPGENC_TABLE_ID, 4);
	table_data(j, luma_dc_bits, ARRAY_SIZE(luma_dc_bits));
	table_data(j, luma_dc_value, ARRAY_SIZE(luma_dc_value));

	jw(j, JPGENC_TAC0_LEN, ARRAY_SIZE(luma_ac_bits) + ARRAY_SIZE(luma_ac_value));
	jw(j, JPGENC_TABLE_ID, 5);
	table_data(j, luma_ac_bits, ARRAY_SIZE(luma_ac_bits));
	table_data(j, luma_ac_value, ARRAY_SIZE(luma_ac_value));

	jw(j, JPGENC_TDC1_LEN, ARRAY_SIZE(chroma_dc_bits) + ARRAY_SIZE(chroma_dc_value));
	jw(j, JPGENC_TABLE_ID, 6);
	table_data(j, chroma_dc_bits, ARRAY_SIZE(chroma_dc_bits));
	table_data(j, chroma_dc_value, ARRAY_SIZE(chroma_dc_value));

	jw(j, JPGENC_TAC1_LEN, ARRAY_SIZE(chroma_ac_bits) + ARRAY_SIZE(chroma_ac_value));
	jw(j, JPGENC_TABLE_ID, 7);
	table_data(j, chroma_ac_bits, ARRAY_SIZE(chroma_ac_bits));
	table_data(j, chroma_ac_value, ARRAY_SIZE(chroma_ac_value));
}

static void qtable_init(struct hisi_jpegenc *j)
{
	table_write(j, 0, luma_qtable1, ARRAY_SIZE(luma_qtable1));
	table_write(j, 1, chroma_qtable1, ARRAY_SIZE(chroma_qtable1));
	table_write(j, 2, luma_qtable2, ARRAY_SIZE(luma_qtable2));
	table_write(j, 3, chroma_qtable2, ARRAY_SIZE(chroma_qtable2));
}

static void hw_init(struct hisi_jpegenc *j)
{
	huff_init(j);
	qtable_init(j);
	jw(j, JPGENC_PREFETCH, 0);		/* smmu bypass: no prefetch */
	jw(j, JPGENC_RESTART, 0);
	jw(j, JPGENC_SYNCCFG, sync_cfg);
	jw(j, 0x90, jpeg_config);
}

static void smmu_bypass(struct hisi_jpegenc *j)
{
	u32 scr, fama;
	int i;

	if (!j->smmu)
		return;

	/*
	 * Match the vendor do_cfg_smmu() bypass path: keep global translation
	 * on (glb_bypass = 0), enable per-SID bypass for the three JPEG stream
	 * IDs, then program the FAMA bypass high bits.  Setting the global
	 * bypass bit alone left the JPGENC reading a fixed address.
	 */
	scr = readl(j->smmu + SMMU_GLOBAL_BYPASS);
	scr &= ~BIT(0);
	scr &= ~(0xf << 16);
	scr |= (5 << 16);		/* ptw_pf, huawei,qos_update = 1 */
	scr &= ~(0x3f << 20);
	scr |= (0x1d << 20);		/* ptw_mid */
	writel(scr, j->smmu + SMMU_GLOBAL_BYPASS);

	for (i = 0; i < 3; i++) {
		u32 id = (i == 0) ? 0 : (i == 1) ? 4 : 5;

		writel(readl(j->smmu + SMMU_SMRX_NS(id)) | BIT(0),
		       j->smmu + SMMU_SMRX_NS(id));
	}

	/* FAMA_CTRL1_NS bits 8..13: bypass MSB (buffers are below 4 GB) */
	fama = readl(j->smmu + 0x224);
	fama &= ~(0x3f << 8);
	writel(fama, j->smmu + 0x224);
}

static void cvdr_config(struct hisi_jpegenc *j, dma_addr_t out, u32 out_size,
			u32 width, u32 height)
{
	u32 last_page = (out + out_size) >> 15;
	u32 start = (out + JPGENC_HEAD_SIZE) >> 4;
	u32 cfg, line, ifcfg;

	/* 34-bit address + 5-bit pixel format (PIX_FMT_D64) */
	cfg = (PIX_FMT_D64 & 0x1f) | BIT(5) | ((last_page & 0x7ffff) << 13);
	writel(cfg, j->cvdr + CVDR_VP_WR_CFG_25);

	/* U_VP_WR_AXI_FS_UPDATE: 30-bit address at bit 2 */
	writel((start & 0x3fffffff) << 2, j->cvdr + CVDR_VP_WR_AXI_FS_25);

	/* U_VP_WR_AXI_LINE_UPDATE_2: stride 11 bits, wstrb at 11, wrap at 15 */
	line = 0x3f | (0xf << 11) | (0x3fff << 15);
	writel(line, j->cvdr + CVDR_VP_WR_AXI_LINE_25);

	ifcfg = readl(j->cvdr + CVDR_VP_WR_IF_CFG_25);
	writel(ifcfg | BIT(31), j->cvdr + CVDR_VP_WR_IF_CFG_25);

	/* access limiter 2 in all four slots */
	writel(0x2222, j->cvdr + CVDR_LIMITER_25);
}

static void cvdr_rd_config(struct hisi_jpegenc *j)
{
	u32 cfg = readl(j->cvdr + CVDR_NR_RD_CFG_4);

	/* allocated DU = 0x0f (huawei,cvdr_limiter_du), enable read port */
	cfg &= ~(0x1f << 5);
	cfg |= (0x0f << 5) | BIT(17) | BIT(18) | BIT(31);
	writel(cfg, j->cvdr + CVDR_NR_RD_CFG_4);

	/* four access limiters = 0x0f, reload = 0x0f */
	writel(0x0fffffff, j->cvdr + CVDR_NR_RD_LIMIT_4);
}


/* ------------------------------------------------------------------ sweep */

static u32 frame_hash(const u8 *out, u32 byte_cnt)
{
	u32 h = 2166136261u;
	u32 n = min_t(u32, byte_cnt, 4096);
	u32 i;

	for (i = 0; i < n; i++)
		h = (h ^ out[JPGENC_HEAD_SIZE + i]) * 16777619u;
	return h;
}

static u32 run_frame(struct hisi_jpegenc *j, u8 *y, u8 *uv,
		     dma_addr_t y_dma, dma_addr_t uv_dma,
		     u8 *out, dma_addr_t out_dma, u32 width, u32 height,
		     bool flat, u32 *byte_cnt)
{
	u32 y_size = width * height;
	u32 stride = (width >> 4) << 4;
	u32 i, bc = 0;

	/* stale bytes at +640 would make an output-offset change look like a hit */
	memset(out + JPGENC_HEAD_SIZE, 0, 4096);
	if (flat)
		memset(y, 0x80, y_size);
	else {
		for (i = 0; i < y_size; i++)
			y[i] = (u8)((i % width) + (i / width));
	}
	memset(uv, 0x80, y_size / 2);

	/* the vendor re-arms the CVDR on every frame, not just at power-on */
	cvdr_config(j, out_dma, JPGENC_OUT_SIZE, width, height);
	cvdr_rd_config(j);

	jw(j, JPGENC_PIC_FORMAT, 1);
	jw(j, JPGENC_INPUT_SWAP, 0);
	jw(j, JPGENC_HRIGHT1, (width - 1) & 0x1fff);
	jw(j, JPGENC_VBOTTOM, (height - 1) & 0x1fff);
	jw(j, JPGENC_HRIGHT2, 0);
	jw(j, JPGENC_TQ_Y, 0);
	jw(j, JPGENC_TQ_U, 1);
	jw(j, JPGENC_TQ_V, 1);
	jw(j, JPGENC_Y_ADDR, (u32)(y_dma >> 4));
	jw(j, JPGENC_UV_ADDR, (u32)(uv_dma >> 4));
	jw(j, JPGENC_STRIDE, (stride >> 4) << 4);
	jw(j, JPGENC_PREREAD, 4);
	/*
	 * huawei,irq-merge = 1: ENCODE_DONE is merged into subctrl 0x118 bit1.
	 * The mask is 0x114 (0x1d = enable) and the clear is 0x110 = bit1.
	 * Without clearing the merged interrupt the JPGENC only encodes the
	 * first frame after power-on; later frames read a stale byte count.
	 */
	writel(0x1d, j->subctrl + 0x114);
	writel(0x2, j->subctrl + 0x110);
	jw(j, JPGENC_STATUS_ICR, 0x30);
	jw(j, JPGENC_ERROR_ICR, 0xff);
	jw(j, JPGENC_ENCODE, 1);
	jw(j, JPGENC_INIT, 1);

	for (i = 0; i < 40; i++) {
		if (readl(j->subctrl + 0x118) & 0x2)
			break;
		if (jr(j, JPGENC_STATUS_RIS) & BIT(4))
			break;
		usleep_range(100, 200);
	}
	bc = jr(j, JPGENC_BYTE_CNT);
	writel(0x2, j->subctrl + 0x110);
	jw(j, JPGENC_STATUS_ICR, 0x30);
	jw(j, JPGENC_ERROR_ICR, 0xff);
	*byte_cnt = bc;
	return frame_hash(out, bc);
}

static bool sweep_skip(u32 reg)
{
	switch (reg) {
	case 0x04:	/* encode */
	case 0x08:	/* init */
	case 0x3c:	/* table data */
	case 0x40:	/* table id */
	case 0x44:
	case 0x48:
	case 0x4c:
	case 0x50:	/* table lengths */
	case 0x6c:
	case 0x70:
	case 0x74:
	case 0x78:
	case 0x7c:
	case 0x80:
	case 0x84:
	case 0x88:
	case 0x8c:	/* interrupt/status */
	case 0x94:
	case 0x98:
	case 0x9c:	/* input addresses/stride */
	case 0xfc:	/* state */
	case 0x108:	/* byte count */
	case 0x130:	/* force clock on */
		return true;
	}
	return false;
}

static const u32 pair_regs[] = {
	0x00, 0x0c, 0x14, 0x18, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38,
	0x60, 0x64, 0x90, 0x100, 0x104, 0x10c, 0x120, 0x124, 0x128, 0x12c,
};

static void sweep_probe(struct hisi_jpegenc *j, u32 reg, u32 old, u32 val,
			u32 b1, u32 b2, u32 h1, u32 h2)
{
	dev_warn(j->dev,
		 "SWEEP HIT reg %#04x %#x -> %#x: grad %u/%08x flat %u/%08x\n",
		 reg, old, val, b1, h1, b2, h2);
}

static int sweep_run(struct hisi_jpegenc *j, u32 width, u32 height)
{
	u32 y_size = width * height;
	u32 uv_size = y_size / 2;
	u8 *yuv, *out;
	dma_addr_t yuv_dma, out_dma;
	u32 b0, b1, h0, h1;
	u32 reg, bit, a, b;
	u32 hits = 0, runs = 0;
	int ret;

	ret = dma_set_mask_and_coherent(j->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	yuv = dma_alloc_coherent(j->dev, y_size + uv_size, &yuv_dma, GFP_KERNEL);
	out = dma_alloc_coherent(j->dev, JPGENC_OUT_SIZE, &out_dma, GFP_KERNEL);
	if (!yuv || !out) {
		ret = -ENOMEM;
		goto out;
	}
	memset(out, 0, JPGENC_OUT_SIZE);

	smmu_bypass(j);
	cvdr_config(j, out_dma, JPGENC_OUT_SIZE, width, height);
	cvdr_rd_config(j);
	hw_init(j);

	/* the first frame after power-on is a warm-up: discard it */
	run_frame(j, yuv, yuv + y_size, yuv_dma, yuv_dma + y_size,
		  out, out_dma, width, height, false, &b0);

	/* baseline: the input path is still broken, so these should match */
	h0 = run_frame(j, yuv, yuv + y_size, yuv_dma, yuv_dma + y_size,
		       out, out_dma, width, height, false, &b0);
	h1 = run_frame(j, yuv, yuv + y_size, yuv_dma, yuv_dma + y_size,
		       out, out_dma, width, height, true, &b1);
	dev_info(j->dev, "sweep baseline: grad %u/%08x flat %u/%08x%s\n",
		 b0, h0, b1, h1, (h0 != h1 || b0 != b1) ? " (already differs!)" : "");

	if (sweep & 1) {
		for (reg = 0; reg <= 0x130; reg += 4) {
			u32 save = jr(j, reg);

			if (sweep_skip(reg))
				continue;
			for (bit = 0; bit < 32; bit++) {
				u32 val = save ^ BIT(bit);
				u32 gb, fb, gh, fh;

				jw(j, reg, val);
				gh = run_frame(j, yuv, yuv + y_size, yuv_dma,
					       yuv_dma + y_size, out, out_dma,
					       width, height, false, &gb);
				fh = run_frame(j, yuv, yuv + y_size, yuv_dma,
					       yuv_dma + y_size, out, out_dma,
					       width, height, true, &fb);
				runs += 2;
				if (gh != fh || gb != fb) {
					sweep_probe(j, reg, save, val,
						    gb, fb, gh, fh);
					hits++;
				}
			}
			jw(j, reg, save);
		}
	}

	if (sweep & 2) {
		hw_init(j);
		for (reg = 0; reg < ARRAY_SIZE(pair_regs); reg++) {
			u32 save = jr(j, pair_regs[reg]);

			for (a = 0; a < 32; a++) {
				for (b = a + 1; b < 32; b++) {
					u32 val = save ^ BIT(a) ^ BIT(b);
					u32 gb, fb, gh, fh;

					jw(j, pair_regs[reg], val);
					gh = run_frame(j, yuv, yuv + y_size,
						       yuv_dma, yuv_dma + y_size,
						       out, out_dma, width, height,
						       false, &gb);
					fh = run_frame(j, yuv, yuv + y_size,
						       yuv_dma, yuv_dma + y_size,
						       out, out_dma, width, height,
						       true, &fb);
					runs += 2;
					if (gh != fh || gb != fb) {
						sweep_probe(j, pair_regs[reg],
							    save, val, gb, fb,
							    gh, fh);
						hits++;
					}
				}
			}
			jw(j, pair_regs[reg], save);
		}
	}

	if (sweep & 4) {
		static const u32 value_regs[] = {
			0x00, 0x0c, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x2c,
			0x30, 0x34, 0x38, 0x60, 0x64, 0x90, 0x100, 0x104,
			0x10c, 0x120, 0x124, 0x128, 0x12c,
		};
		u32 ri;

		hw_init(j);
		for (ri = 0; ri < ARRAY_SIZE(value_regs); ri++) {
			u32 r = value_regs[ri];
			u32 save = jr(j, r);
			u32 v;

			if (sweep_reg != 0xffffffff && r != sweep_reg)
				continue;
			dev_info(j->dev, "value sweep reg %#04x (save %#x)\n",
				 r, save);
			for (v = 0; v <= sweep_max; v++) {
				u32 gb, fb, gh, fh;

				jw(j, r, v);
				gh = run_frame(j, yuv, yuv + y_size, yuv_dma,
					       yuv_dma + y_size, out, out_dma,
					       width, height, false, &gb);
				fh = run_frame(j, yuv, yuv + y_size, yuv_dma,
					       yuv_dma + y_size, out, out_dma,
					       width, height, true, &fb);
				runs += 2;
				if (gh != fh || gb != fb) {
					sweep_probe(j, r, save, v, gb, fb,
						    gh, fh);
					hits++;
				}
			}
			jw(j, r, save);
			dev_info(j->dev, "value sweep reg %#04x done\n", r);
		}
	}

	if (sweep & 8) {
		u32 r;

		if (cvdr_end > 0x1ffc)
			cvdr_end = 0x1ffc;
		cvdr_start &= ~3u;
		hw_init(j);
		dev_info(j->dev, "cvdr sweep %#x-%#x\n", cvdr_start, cvdr_end);
		for (r = cvdr_start; r <= cvdr_end; r += 4) {
			u32 save = readl(j->cvdr + r);
			u32 bit;

			for (bit = 0; bit < 32; bit++) {
				u32 val = save ^ BIT(bit);
				u32 gb, fb, gh, fh;

				writel(val, j->cvdr + r);
				gh = run_frame(j, yuv, yuv + y_size, yuv_dma,
					       yuv_dma + y_size, out, out_dma,
					       width, height, false, &gb);
				fh = run_frame(j, yuv, yuv + y_size, yuv_dma,
					       yuv_dma + y_size, out, out_dma,
					       width, height, true, &fb);
				runs += 2;
				if (gh != fh || gb != fb) {
					sweep_probe(j, r, save, val, gb, fb,
						    gh, fh);
					hits++;
				}
			}
			writel(save, j->cvdr + r);
		}
		dev_info(j->dev, "cvdr sweep %#x-%#x done\n",
			 cvdr_start, cvdr_end);
	}

	dev_info(j->dev, "sweep done: %u frames, %u hits\n", runs, hits);
	ret = 0;
out:
	if (out)
		dma_free_coherent(j->dev, JPGENC_OUT_SIZE, out, out_dma);
	if (yuv)
		dma_free_coherent(j->dev, y_size + uv_size, yuv, yuv_dma);
	return ret;
}

static int encode_test(struct hisi_jpegenc *j, u32 width, u32 height)
{
	u32 y_size = width * height;
	u32 uv_size = y_size / 2;
	u8 *yuv, *out;
	dma_addr_t yuv_dma, out_dma;
	u32 bc = 0, h;
	int ret;

	ret = dma_set_mask_and_coherent(j->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	yuv = dma_alloc_coherent(j->dev, y_size + uv_size, &yuv_dma, GFP_KERNEL);
	out = dma_alloc_coherent(j->dev, JPGENC_OUT_SIZE, &out_dma, GFP_KERNEL);
	if (!yuv || !out) {
		ret = -ENOMEM;
		goto out;
	}
	memset(out, 0, JPGENC_OUT_SIZE);
	memset(out, 0, JPGENC_OUT_SIZE);

	smmu_bypass(j);
	cvdr_config(j, out_dma, JPGENC_OUT_SIZE, width, height);
	cvdr_rd_config(j);
	hw_init(j);

	if (seq) {
		static const bool pat[] = { 0, 0, 1, 1, 0, 0 };
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(pat); i++) {
			h = run_frame(j, yuv, yuv + y_size, yuv_dma,
				      yuv_dma + y_size, out, out_dma,
				      width, height, pat[i], &bc);
			dev_info(j->dev,
				 "seq %u flat=%u bytecnt %u hash %08x\n",
				 i, pat[i], bc, h);
		}
	} else {
		h = run_frame(j, yuv, yuv + y_size, yuv_dma,
			      yuv_dma + y_size, out, out_dma, width, height,
			      flat, &bc);
	}
	dev_info(j->dev,
		 "encode done: bytecnt %u hash %08x status %#x error %#x first %02x %02x %02x %02x\n",
		 bc, h, jr(j, JPGENC_STATUS_RIS), jr(j, JPGENC_ERROR_RIS),
		 out[JPGENC_HEAD_SIZE], out[JPGENC_HEAD_SIZE + 1],
		 out[JPGENC_HEAD_SIZE + 2], out[JPGENC_HEAD_SIZE + 3]);

	if (bc) {
		j->out = out;
		j->out_dma = out_dma;
		j->out_size = JPGENC_OUT_SIZE;
		j->byte_cnt = bc;
		j->have_image = true;
		out = NULL;
		ret = 0;
	} else {
		ret = -EIO;
	}

out:
	if (out)
		dma_free_coherent(j->dev, JPGENC_OUT_SIZE, out, out_dma);
	if (yuv)
		dma_free_coherent(j->dev, y_size + uv_size, yuv, yuv_dma);
	return ret;
}

static int hisi_jpegenc_power_on(struct hisi_jpegenc *j)
{
	u32 clk_ctl_offset = 0;
	u32 before, after;
	int ret;

	of_property_read_u32(j->dev->of_node, "huawei,clk_ctl_offset",
			     &clk_ctl_offset);

	ret = regulator_enable(j->media_supply);
	if (ret)
		return dev_err_probe(j->dev, ret, "media supply\n");

	ret = clk_set_rate(j->clk_func, 600000000);
	if (ret)
		dev_warn(j->dev, "clk_jpg_func 600 MHz failed: %d\n", ret);
	ret = clk_prepare_enable(j->clk_func);
	if (ret)
		goto err_media;

	ret = regulator_enable(j->srt_supply);
	if (ret)
		goto err_clk;

	if (j->aclk)
		clk_prepare_enable(j->aclk);
	if (j->pclk)
		clk_prepare_enable(j->pclk);

	before = readl(j->subctrl + clk_ctl_offset);
	writel(before | BIT(0), j->subctrl + clk_ctl_offset);
	after = readl(j->subctrl + clk_ctl_offset);
	dev_info(j->dev, "subctrl+%#x: %#x -> %#x (clk %lu Hz)\n",
		 clk_ctl_offset, before, after, clk_get_rate(j->clk_func));

	jw(j, JPGENC_FORCE_CLK_ON, 1);
	before = jr(j, JPGENC_RESTART);
	jw(j, JPGENC_RESTART, 0x5678);
	after = jr(j, JPGENC_RESTART);
	dev_info(j->dev, "JPGENC scratch %#x -> %#x, state %#x bytecnt %#x\n",
		 before, after, jr(j, 0xfc), jr(j, JPGENC_BYTE_CNT));

	j->powered = true;
	return (after & 0xffff) == 0x5678 ? 0 : -EIO;

err_clk:
	clk_disable_unprepare(j->clk_func);
err_media:
	regulator_disable(j->media_supply);
	return ret;
}

static void hisi_jpegenc_power_off(struct hisi_jpegenc *j)
{
	if (!j->powered)
		return;
	jw(j, JPGENC_FORCE_CLK_ON, 0);
	if (j->pclk)
		clk_disable_unprepare(j->pclk);
	if (j->aclk)
		clk_disable_unprepare(j->aclk);
	regulator_disable(j->srt_supply);
	clk_disable_unprepare(j->clk_func);
	regulator_disable(j->media_supply);
	j->powered = false;
}

static int hisi_jpegenc_raw_show(struct seq_file *m, void *unused)
{
	struct hisi_jpegenc *j = m->private;
	u32 i;

	if (!j->have_image) {
		seq_puts(m, "no image\n");
		return 0;
	}
	for (i = 0; i < j->byte_cnt; i++)
		seq_printf(m, "%02x", j->out[JPGENC_HEAD_SIZE + i]);
	seq_putc(m, '\n');
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(hisi_jpegenc_raw);

static int hisi_jpegenc_head_show(struct seq_file *m, void *unused)
{
	struct hisi_jpegenc *j = m->private;
	u32 i;

	if (!j->have_image) {
		seq_puts(m, "no image\n");
		return 0;
	}
	for (i = 0; i < 128; i++)
		seq_printf(m, "%02x", j->out[i]);
	seq_putc(m, '\n');
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(hisi_jpegenc_head);

static int hisi_jpegenc_probe(struct platform_device *pdev)
{
	struct hisi_jpegenc *j;
	u32 base[2];
	int ret;

	j = devm_kzalloc(&pdev->dev, sizeof(*j), GFP_KERNEL);
	if (!j)
		return -ENOMEM;
	j->dev = &pdev->dev;

	j->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(j->regs))
		return PTR_ERR(j->regs);

	if (of_property_read_u32_array(pdev->dev.of_node, "huawei,subctrl-base",
				       base, 2))
		return -EINVAL;
	j->subctrl = devm_ioremap(&pdev->dev, base[0], base[1]);
	if (!j->subctrl)
		return -ENOMEM;

	j->cvdr = devm_ioremap(&pdev->dev, 0xe8006000, 0x2000);
	j->smmu = devm_ioremap(&pdev->dev, 0xe8020000, 0x20000);
	if (!j->cvdr || !j->smmu)
		return -ENOMEM;

	j->media_supply = devm_regulator_get(&pdev->dev, "hjpeg-media");
	if (IS_ERR(j->media_supply))
		return dev_err_probe(&pdev->dev, PTR_ERR(j->media_supply),
				     "hjpeg-media supply\n");
	j->srt_supply = devm_regulator_get(&pdev->dev, "hjpeg-srt");
	if (IS_ERR(j->srt_supply))
		return dev_err_probe(&pdev->dev, PTR_ERR(j->srt_supply),
				     "hjpeg-srt supply\n");

	j->clk_func = devm_clk_get(&pdev->dev, "clk_jpg_func");
	if (IS_ERR(j->clk_func))
		return dev_err_probe(&pdev->dev, PTR_ERR(j->clk_func),
				     "clk_jpg_func\n");
	j->aclk = devm_clk_get_optional(&pdev->dev, "aclk_jpg");
	if (IS_ERR(j->aclk))
		return PTR_ERR(j->aclk);
	j->pclk = devm_clk_get_optional(&pdev->dev, "pclk_jpg");
	if (IS_ERR(j->pclk))
		return PTR_ERR(j->pclk);

	platform_set_drvdata(pdev, j);
	j->dbg = debugfs_create_file("hisi-jpegenc", 0444, NULL, j,
				     &hisi_jpegenc_raw_fops);
	debugfs_create_file("hisi-jpegenc-head", 0444, NULL, j,
			    &hisi_jpegenc_head_fops);

	if (!do_probe) {
		dev_info(&pdev->dev, "do_probe=0: resources mapped only\n");
		return 0;
	}

	ret = hisi_jpegenc_power_on(j);
	if (ret) {
		dev_warn(&pdev->dev, "power-on/register test failed: %d\n", ret);
		return 0;
	}

	if (sweep) {
		sweep_run(j, 320, 240);
		return 0;
	}

	if (test)
		encode_test(j, 320, 240);

	return 0;
}

static void hisi_jpegenc_remove(struct platform_device *pdev)
{
	struct hisi_jpegenc *j = platform_get_drvdata(pdev);

	debugfs_remove(j->dbg);
	if (j->out)
		dma_free_coherent(j->dev, j->out_size, j->out, j->out_dma);
	hisi_jpegenc_power_off(j);
}

static struct platform_driver hisi_jpegenc_driver = {
	.probe = hisi_jpegenc_probe,
	.remove = hisi_jpegenc_remove,
	.driver = {
		.name = "hisi-jpegenc",
		.of_match_table = hisi_jpegenc_of_match,
	},
};
module_platform_driver(hisi_jpegenc_driver);

MODULE_DESCRIPTION("HiSilicon Kirin 990 hardware JPEG encoder bring-up");
MODULE_LICENSE("GPL");
