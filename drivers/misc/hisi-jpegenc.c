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
	jw(j, JPGENC_SYNCCFG, BIT(0) | BIT(2));	/* source = external, sram noopt */
}

static void smmu_bypass(struct hisi_jpegenc *j)
{
	int i;

	if (!j->smmu)
		return;
	/* global bypass plus the three JPEG stream IDs, like the vendor driver */
	writel(readl(j->smmu + SMMU_GLOBAL_BYPASS) | BIT(0),
	       j->smmu + SMMU_GLOBAL_BYPASS);
	for (i = 0; i < 3; i++) {
		u32 id = (i == 0) ? 0 : (i == 1) ? 4 : 5;

		writel(readl(j->smmu + SMMU_SMRX_NS(id)) | BIT(0),
		       j->smmu + SMMU_SMRX_NS(id));
	}
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

static int encode_test(struct hisi_jpegenc *j, u32 width, u32 height)
{
	u32 y_size = width * height;
	u32 uv_size = y_size / 2;
	u8 *yuv, *y, *uv, *out;
	dma_addr_t yuv_dma, y_dma, uv_dma, out_dma;
	u32 stride = (width >> 4) << 4;
	u32 i, byte_cnt = 0;
	int ret = -ETIMEDOUT;

	ret = dma_set_mask_and_coherent(j->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	yuv = dma_alloc_coherent(j->dev, y_size + uv_size, &yuv_dma, GFP_KERNEL);
	out = dma_alloc_coherent(j->dev, JPGENC_OUT_SIZE, &out_dma, GFP_KERNEL);
	y = yuv;
	uv = yuv ? yuv + y_size : NULL;
	y_dma = yuv_dma;
	uv_dma = yuv_dma + y_size;
	if (!yuv || !out) {
		ret = -ENOMEM;
		goto free;
	}

	if (flat) {
		memset(y, 0x80, y_size);
	} else {
		for (i = 0; i < y_size; i++)
			y[i] = (u8)((i % width) + (i / width));
	}
	memset(uv, 0x80, uv_size);
	memset(out, 0, JPGENC_OUT_SIZE);

	dev_info(j->dev, "encode %ux%u y %pad uv %pad out %pad\n",
		 width, height, &y_dma, &uv_dma, &out_dma);

	smmu_bypass(j);
	cvdr_config(j, out_dma, JPGENC_OUT_SIZE, width, height);
	cvdr_rd_config(j);
	hw_init(j);

	/* NV12 / YUV420 */
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
	jw(j, JPGENC_ENCODE, 1);
	jw(j, JPGENC_INIT, 1);

	for (i = 0; i < 500; i++) {
		byte_cnt = jr(j, JPGENC_BYTE_CNT);
		if (byte_cnt)
			break;
		usleep_range(1000, 2000);
	}

	dev_info(j->dev,
		 "encode done: bytecnt %u status %#x error %#x state %#x first %02x %02x %02x %02x\n",
		 byte_cnt, jr(j, JPGENC_STATUS_RIS), jr(j, JPGENC_ERROR_RIS),
		 jr(j, JPGENC_FORCE_CLK_ON),
		 out[JPGENC_HEAD_SIZE], out[JPGENC_HEAD_SIZE + 1],
		 out[JPGENC_HEAD_SIZE + 2], out[JPGENC_HEAD_SIZE + 3]);
	dev_info(j->dev,
		 "JPGENC: en %#x init %#x fmt %#x h %#x v %#x y %#x uv %#x stride %#x pre %#x sync %#x\n",
		 jr(j, JPGENC_ENCODE), jr(j, JPGENC_INIT), jr(j, JPGENC_PIC_FORMAT),
		 jr(j, JPGENC_HRIGHT1), jr(j, JPGENC_VBOTTOM), jr(j, JPGENC_Y_ADDR),
		 jr(j, JPGENC_UV_ADDR), jr(j, JPGENC_STRIDE), jr(j, JPGENC_PREREAD),
		 jr(j, JPGENC_SYNCCFG));
	dev_info(j->dev,
		 "CVDR: cfg %#x fs %#x line %#x if %#x dbg %#x/%#x\n",
		 readl(j->cvdr + CVDR_VP_WR_CFG_25),
		 readl(j->cvdr + CVDR_VP_WR_AXI_FS_25),
		 readl(j->cvdr + CVDR_VP_WR_AXI_LINE_25),
		 readl(j->cvdr + CVDR_VP_WR_IF_CFG_25),
		 readl(j->cvdr + 0xf00), readl(j->cvdr + 0x1da4));
	dev_info(j->dev,
		 "CVDR rd: cfg %#x lim %#x\n",
		 readl(j->cvdr + CVDR_NR_RD_CFG_4),
		 readl(j->cvdr + CVDR_NR_RD_LIMIT_4));
	dev_info(j->dev,
		 "SMMU: bypass %#x int %#x rd %#x/%#x wr %#x/%#x msb %#x\n",
		 readl(j->smmu + SMMU_GLOBAL_BYPASS),
		 readl(j->smmu + 0x18),
		 readl(j->smmu + 0x304), readl(j->smmu + 0x300),
		 readl(j->smmu + 0x308), readl(j->smmu + 0x300),
		 readl(j->smmu + 0x300));
	if (byte_cnt)
		ret = 0;

free:
	if (ret == 0) {
		j->out = out;
		j->out_dma = out_dma;
		j->out_size = JPGENC_OUT_SIZE;
		j->byte_cnt = byte_cnt;
		j->have_image = true;
		out = NULL;
	}
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

	if (!do_probe) {
		dev_info(&pdev->dev, "do_probe=0: resources mapped only\n");
		return 0;
	}

	ret = hisi_jpegenc_power_on(j);
	if (ret) {
		dev_warn(&pdev->dev, "power-on/register test failed: %d\n", ret);
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
