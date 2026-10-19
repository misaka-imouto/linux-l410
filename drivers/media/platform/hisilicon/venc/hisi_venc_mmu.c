// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video encoder: private SMMU
 *
 * The VENC block has its own private SMMU (a common control block at +0x20000
 * and master interface block at +0x10000). It walks an ARMv7 LPAE stage-1 table
 * with 32-bit input address space (ARM_32_LPAE_S1) matching the io-pgtable API.
 */

#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "hisi_venc.h"

#define HIVENC_IOVA_START	0x00100000UL
#define HIVENC_IOVA_END		0xe0000000UL

struct hivenc_deferred_range {
	struct list_head node;
	unsigned long iova;
	size_t size;
};

static void hivenc_tlb_flush_all(void *cookie)
{
}

static void hivenc_tlb_flush_walk(unsigned long iova, size_t size,
				  size_t granule, void *cookie)
{
}

static const struct iommu_flush_ops hivenc_flush_ops = {
	.tlb_flush_all = hivenc_tlb_flush_all,
	.tlb_flush_walk = hivenc_tlb_flush_walk,
};

int hivenc_aux_alloc(struct hivenc_dev *venc, struct hivenc_aux_buf *buf,
		     size_t size)
{
	buf->size = PAGE_ALIGN(size);
	buf->vaddr = dma_alloc_coherent(venc->dev, buf->size, &buf->dma,
					GFP_KERNEL);
	if (!buf->vaddr)
		return -ENOMEM;
	return 0;
}

void hivenc_aux_free(struct hivenc_dev *venc, struct hivenc_aux_buf *buf)
{
	if (buf->vaddr) {
		dma_free_coherent(venc->dev, buf->size, buf->vaddr, buf->dma);
		buf->vaddr = NULL;
		buf->dma = 0;
		buf->size = 0;
	}
}

int hivenc_mmu_init(struct hivenc_dev *venc)
{
	struct hivenc_mmu *mmu = &venc->mmu;
	int ret;

	mutex_init(&mmu->lock);
	INIT_LIST_HEAD(&mmu->deferred);

	mmu->cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= SZ_4K | SZ_2M | SZ_1G,
		.ias		= 32,
		.oas		= 40,
		.coherent_walk	= false,
		.tlb		= &hivenc_flush_ops,
		.iommu_dev	= venc->dev,
		.quirks		= IO_PGTABLE_QUIRK_ARM_NS,
	};
	mmu->ops = alloc_io_pgtable_ops(ARM_32_LPAE_S1, &mmu->cfg, venc);
	if (!mmu->ops)
		return -ENOMEM;

	mmu->iova_pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!mmu->iova_pool) {
		ret = -ENOMEM;
		goto err_pgtable;
	}
	ret = gen_pool_add(mmu->iova_pool, HIVENC_IOVA_START,
			   HIVENC_IOVA_END - HIVENC_IOVA_START, -1);
	if (ret)
		goto err_pool;

	/* Scratch buffer pages the SMMU accesses on translation faults */
	ret = hivenc_aux_alloc(venc, &mmu->err_rd, PAGE_SIZE);
	if (ret)
		goto err_pool;
	ret = hivenc_aux_alloc(venc, &mmu->err_wr, PAGE_SIZE);
	if (ret)
		goto err_rd;

	return 0;

err_rd:
	hivenc_aux_free(venc, &mmu->err_rd);
err_pool:
	gen_pool_destroy(mmu->iova_pool);
err_pgtable:
	free_io_pgtable_ops(mmu->ops);
	return ret;
}

void hivenc_mmu_fini(struct hivenc_dev *venc)
{
	struct hivenc_mmu *mmu = &venc->mmu;

	hivenc_aux_free(venc, &mmu->err_wr);
	hivenc_aux_free(venc, &mmu->err_rd);
	hivenc_mmu_powered_off(venc);
	gen_pool_destroy(mmu->iova_pool);
	free_io_pgtable_ops(mmu->ops);
}

void hivenc_mmu_hw_setup(struct hivenc_dev *venc)
{
	struct hivenc_mmu *mmu = &venc->mmu;
	u64 ttbr = mmu->cfg.arm_lpae_s1_cfg.ttbr;
	u32 scr;
	int i;

	scr = vedu_read(venc, SMMU_SCR);
	scr &= ~BIT(0);			/* global bypass off */
	scr |= BIT(1) | BIT(2);		/* rqos_en, wqos_en */
	vedu_write(venc, SMMU_SCR, scr);

	/* Stream matching */
	for (i = 0; i < 32; i++)
		vedu_write(venc, SMMU_SMRX_NS(i), (i & 1) ? 0x1d : 0x1c);

	vedu_write(venc, SMMU_CB_TTBR0, lower_32_bits(ttbr));
	vedu_write(venc, SMMU_CB_TTBR_MSB, upper_32_bits(ttbr) & 0xffff);
	vedu_write(venc, SMMU_CB_TTBCR, vedu_read(venc, SMMU_CB_TTBCR) | BIT(0));

	vedu_write(venc, SMMU_ERR_RDADDR_NS, lower_32_bits(mmu->err_rd.dma));
	vedu_write(venc, SMMU_ERR_WRADDR_NS, lower_32_bits(mmu->err_wr.dma));
	vedu_write(venc, SMMU_ERR_ADDR_MSB_NS,
		   (upper_32_bits(mmu->err_rd.dma) & 0xffff) |
		   ((upper_32_bits(mmu->err_wr.dma) & 0xffff) << 16));

	for (i = 0; i < 4; i++)
		vedu_write(venc, SMMU_MSTR_SMRX(i), 0);
}

void hivenc_mmu_powered_off(struct hivenc_dev *venc)
{
	struct hivenc_mmu *mmu = &venc->mmu;
	struct hivenc_deferred_range *r, *tmp;

	mutex_lock(&mmu->lock);
	list_for_each_entry_safe(r, tmp, &mmu->deferred, node) {
		gen_pool_free(mmu->iova_pool, r->iova, r->size);
		list_del(&r->node);
		kfree(r);
	}
	mutex_unlock(&mmu->lock);
}

static void hivenc_unmap_range(struct io_pgtable_ops *ops, unsigned long iova,
			       size_t size)
{
	while (size) {
		size_t n = ops->unmap_pages(ops, iova, SZ_4K, size / SZ_4K, NULL);

		if (WARN_ON(!n))
			break;
		iova += n;
		size -= n;
	}
}

int hivenc_mmu_map_sgt(struct hivenc_dev *venc, struct sg_table *sgt,
		       size_t size, struct hivenc_mapping *map)
{
	struct hivenc_mmu *mmu = &venc->mmu;
	struct io_pgtable_ops *ops = mmu->ops;
	struct scatterlist *sg;
	unsigned long iova, cur;
	size_t mapped_total = 0;
	int i, ret = 0;

	size = PAGE_ALIGN(size);
	iova = gen_pool_alloc(mmu->iova_pool, size);
	if (!iova)
		return -ENOSPC;

	mutex_lock(&mmu->lock);
	cur = iova;
	for_each_sgtable_sg(sgt, sg, i) {
		phys_addr_t pa = sg_phys(sg);
		size_t len = sg->length, mapped = 0;

		if (WARN_ON(!PAGE_ALIGNED(pa) || !PAGE_ALIGNED(len))) {
			ret = -EINVAL;
			break;
		}
		len = min_t(size_t, len, size - mapped_total);
		if (!len)
			break;

		while (len) {
			mapped = 0;
			ret = ops->map_pages(ops, cur, pa, SZ_4K, len / SZ_4K,
					     IOMMU_READ | IOMMU_WRITE, GFP_KERNEL,
					     &mapped);
			cur += mapped;
			pa += mapped;
			len -= mapped;
			mapped_total += mapped;
			if (ret)
				break;
		}
		if (ret)
			break;
	}

	if (ret) {
		hivenc_unmap_range(ops, iova, mapped_total);
		gen_pool_free(mmu->iova_pool, iova, size);
		mutex_unlock(&mmu->lock);
		return ret;
	}
	mutex_unlock(&mmu->lock);

	map->iova = iova;
	map->size = size;
	return 0;
}

void hivenc_mmu_unmap(struct hivenc_dev *venc, struct hivenc_mapping *map)
{
	struct hivenc_mmu *mmu = &venc->mmu;
	struct hivenc_deferred_range *r;

	if (!map->iova)
		return;

	mutex_lock(&mmu->lock);
	hivenc_unmap_range(mmu->ops, map->iova, map->size);

	r = kmalloc(sizeof(*r), GFP_KERNEL);
	if (r) {
		r->iova = map->iova;
		r->size = map->size;
		list_add(&r->node, &mmu->deferred);
	} else {
		gen_pool_free(mmu->iova_pool, map->iova, map->size);
	}
	mutex_unlock(&mmu->lock);

	map->iova = 0;
	map->size = 0;
}
