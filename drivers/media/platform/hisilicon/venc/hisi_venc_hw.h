/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HiSilicon Kirin 990 (VCodec V500, VEDU V500R003) video encoder hardware registers
 *
 * Register definitions, bitfields, and constants for the VEDU video encoding block.
 */

#ifndef HISI_VENC_HW_H_
#define HISI_VENC_HW_H_

#include <linux/bits.h>
#include <linux/types.h>

#include "hisi_venc_regs.h"

/* VEDU Core Registers (Offsets from venc core base) */
#define VEDU_VCPI_INTMASK		0x0000
#define VEDU_VCPI_INTCLR		0x0004
#define VEDU_VCPI_START			0x0008
#define VEDU_VCPI_CNTCLR		0x000c
#define VEDU_VCPI_FRAMENO		0x0010
#define VEDU_VCPI_BP_POS		0x0014
#define VEDU_VCPI_TIMEOUT		0x0018
#define VEDU_VCPI_MODE			0x001c
#define VEDU_VCPI_TILE_SIZE		0x0020
#define VEDU_VCPI_PICSIZE_PIX		0x0024
#define VEDU_VCPI_MULTISLC		0x0028
#define VEDU_VCPI_QPCFG			0x002c
#define VEDU_VCPI_DBLKCFG		0x0030
#define VEDU_VCPI_LOW_POWER		0x0034
#define VEDU_VCPI_OUTSTD		0x0038
#define VEDU_VCPI_TMV_LOAD		0x003c
#define VEDU_VCPI_CROSS_TILE_SLC	0x0040
#define VEDU_VCPI_REF_FLAG		0x0064

/* Source and reference buffers */
#define VEDU_VCPI_TUNLCELL_ADDR		0x00e8
#define VEDU_VCPI_SRC_YADDR		0x00ec
#define VEDU_VCPI_SRC_CADDR		0x00f0
#define VEDU_VCPI_SRC_VADDR		0x00f4
#define VEDU_VCPI_STRIDE		0x0100
#define VEDU_VCPI_REC_YADDR		0x0104
#define VEDU_VCPI_REC_CADDR		0x0108
#define VEDU_VCPI_REC_STRIDE		0x010c
#define VEDU_VCPI_REFY_L0_ADDR		0x0118
#define VEDU_VCPI_REFC_L0_ADDR		0x011c
#define VEDU_VCPI_REF_L0_STRIDE		0x0120

/* Stream output address and format */
#define VEDU_VCPI_STRMADDR		0x0164
#define VEDU_VCPI_STRFMT		0x0174

/* Rate control & QPG registers */
#define VEDU_QPG_LAMBDA_MODE		0x0a84
#define VEDU_QPG_QP_RESTRAIN		0x0a88
#define VEDU_QPG_MAX_MIN_QP		0x0a90
#define VEDU_QPG_AVERAGE_LCU_BITS	0x0a94
#define VEDU_QPG_ROW_TARGET_BITS	0x0a98
#define VEDU_VCTRL_INTRA_RDO_FACTOR_0	0x0a9c
#define VEDU_VCTRL_INTRA_RDO_FACTOR_1	0x0aa0
#define VEDU_VCTRL_INTRA_RDO_FACTOR_2	0x0aa4
#define VEDU_PME_NEW_COST		0x0aa8

/* Stream buffer length & address arrays (slot 0..15) */
#define VEDU_VLCST_STRMBUFLEN(n)	(0x1c0c + 4 * (n))
#define VEDU_VLCST_STRMADDR(n)		(0x1c4c + 4 * (n))

/* CABAC / VLC configuration */
#define VEDU_CABAC_GLB_CFG		0x1c8c
#define VEDU_CABAC_SLCHDR_SIZE		0x1c90
#define VEDU_CABAC_SLCHDR_PART1		0x1c94
#define VEDU_VCPI_VLC_CONFIG		0x1c98
#define VEDU_CABAC_SLCHDR_PART2(n)	(0x1c9c + 4 * (n)) /* n=0..7 */
#define VEDU_VLC_SLCHDRSTRM(n)		(0x1ce4 + 4 * (n)) /* n=0..3 */
#define VEDU_VLC_SLCHDRPARA		0x1d08

/* Interrupt and status registers */
#define FUNC_VCPI_INTSTAT		0x3000
#define FUNC_VCPI_RAWINT		0x3004
#define FUNC_VCPI_VEDU_TIMER		0x3008
#define FUNC_CABAC_PIC_STRMSIZE		0x37c0
#define VLC_PIC_TTBITS			0x37c4

/* MMU pre-fetch address bounds */
#define MMU_PRE_SRC_YADDR_STR		0x80a8
#define MMU_PRE_SRC_YADDR_END		0x80ac
#define MMU_PRE_SRC_CADDR_STR		0x80b0
#define MMU_PRE_SRC_CADDR_END		0x80b4
#define MMU_PRE_STRMADDR0_STR		0x80d0
#define MMU_PRE_STRMADDR0_END		0x80d4

/*
 * SMMU registers, from the vendor venc_hivna/drv/product/HiVCodecV500/smmu.c and
 * smmu_regs.h: common block at +0x20000, master block at +0x1A000.
 */
#define SMMU_COMMON_BASE		0x20000
#define SMMU_SCR			(SMMU_COMMON_BASE + 0x000)
#define SMMU_INTMASK_NS			(SMMU_COMMON_BASE + 0x010)
#define SMMU_INTSTAT_NS			(SMMU_COMMON_BASE + 0x018)
#define SMMU_INTCLR_NS			(SMMU_COMMON_BASE + 0x01c)
#define SMMU_SMRX_NS(n)			(SMMU_COMMON_BASE + 0x020 + 4 * (n))
#define SMMU_CB_TTBR0			(SMMU_COMMON_BASE + 0x204)
#define SMMU_CB_TTBR1			(SMMU_COMMON_BASE + 0x208)
#define SMMU_CB_TTBCR			(SMMU_COMMON_BASE + 0x20c)
#define SMMU_CB_TTBR_MSB		(SMMU_COMMON_BASE + 0x224)
#define SMMU_ERR_ADDR_MSB_NS		(SMMU_COMMON_BASE + 0x300)
#define SMMU_ERR_RDADDR_NS		(SMMU_COMMON_BASE + 0x304)
#define SMMU_ERR_WRADDR_NS		(SMMU_COMMON_BASE + 0x308)

#define SMMU_MSTR_BASE			0x1a000
#define SMMU_MSTR_GLB_BYPASS		(SMMU_MSTR_BASE + 0x000)
#define SMMU_MSTR_DEBUG(n)		(SMMU_MSTR_BASE + (n))
#define SMMU_MSTR_SMRX_0(n)		(SMMU_MSTR_BASE + 0x100 + 4 * (n))
#define SMMU_MSTR_SMRX_1(n)		(SMMU_MSTR_BASE + 0x260 + 4 * (n))
#define SMMU_MSTR_SMRX_2(n)		(SMMU_MSTR_BASE + 0x3c0 + 4 * (n))
#define SMMU_MSTR_SMR_NUM		37

/* Bit definitions for VEDU_VCPI_START */
#define VCPI_START_VSTART		BIT(0)
#define VCPI_START_VSTEP		BIT(1)

/* Bit definitions for FUNC_VCPI_RAWINT & VEDU_VCPI_INTMASK */
#define RAWINT_VE_EOP			BIT(0) /* End of Picture / encode done */
#define RAWINT_VEDU_SLC_END		BIT(1) /* Slice end */
#define RAWINT_VE_BUFFULL		BIT(2) /* Stream buffer full */
#define RAWINT_VE_PBITSOVER		BIT(3) /* Picture bits overflow */
#define RAWINT_VEDU_TIMEOUT		BIT(6) /* Hardware timeout */
#define RAWINT_CFG_ERR			BIT(7) /* Configuration error */

/* Protocol values */
enum hivenc_protocol {
	HIVENC_PROTO_HEVC = 0,
	HIVENC_PROTO_H264 = 1,
};

/* Frame type values */
enum hivenc_frame_type {
	HIVENC_FTYPE_I = 0,
	HIVENC_FTYPE_P = 1,
	HIVENC_FTYPE_B = 2,
};

/* LCU size */
enum hivenc_lcu_size {
	HIVENC_LCU_16 = 0,
	HIVENC_LCU_32 = 1,
	HIVENC_LCU_64 = 2,
};

/* Helper for constructing VEDU_VCPI_MODE */
static inline u32 hivenc_build_mode(enum hivenc_protocol proto,
				    enum hivenc_frame_type ftype,
				    bool is_idr,
				    bool is_cabac)
{
	u32 val = 0;

	/* bit 1: vcpi_lcu_time_sel */
	val |= BIT(1);
	/* bit 3..2: vcpi_protocol */
	val |= ((u32)proto & 0x3) << 2;
	/* bit 5: vcpi_slice_int_en */
	val |= BIT(5);
	/* bit 6..7: sao luma/chroma (for HEVC) */
	if (proto == HIVENC_PROTO_HEVC)
		val |= BIT(6) | BIT(7);
	/* bit 11..10: vcpi_frame_type */
	val |= ((u32)ftype & 0x3) << 10;
	/* bit 12: vcpi_entropy_mode (1 for CABAC, 0 for CAVLC) */
	if (is_cabac)
		val |= BIT(12);
	/* bit 14: vcpi_ref_num (0 = 1 ref frame for P) */
	val |= (0 & 0x1) << 14;
	/* bit 16: vcpi_idr_pic */
	if (is_idr)
		val |= BIT(16);
	/* bit 17: vcpi_pskip_en */
	val |= BIT(17);
	/* bit 27..26: vcpi_lcu_size (0=16 for H.264, 2=64 for HEVC) */
	val |= ((proto == HIVENC_PROTO_HEVC ? HIVENC_LCU_64 : HIVENC_LCU_16) & 0x3) << 26;

	return val;
}

#endif /* HISI_VENC_HW_H_ */
