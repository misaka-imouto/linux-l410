// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video encoder: HEVC (H.265) codec support
 *
 * Implements HEVC VPS/SPS/PPS generation and hardware slice header configuration.
 */

#include <linux/kernel.h>
#include <linux/string.h>

#include "hisi_venc.h"

struct bitstream {
	u8 *buf;
	size_t size;
	size_t bit_pos;
};

static void bs_init(struct bitstream *bs, u8 *buf, size_t size)
{
	bs->buf = buf;
	bs->size = size;
	bs->bit_pos = 0;
	if (size > 0)
		memset(buf, 0, size);
}

static void bs_put_bit(struct bitstream *bs, u32 bit)
{
	size_t byte_idx = bs->bit_pos >> 3;
	size_t bit_idx = 7 - (bs->bit_pos & 7);

	if (byte_idx < bs->size) {
		if (bit)
			bs->buf[byte_idx] |= (1 << bit_idx);
		else
			bs->buf[byte_idx] &= ~(1 << bit_idx);
	}
	bs->bit_pos++;
}

static void bs_put_bits(struct bitstream *bs, u32 val, int n)
{
	int i;

	for (i = n - 1; i >= 0; i--)
		bs_put_bit(bs, (val >> i) & 1);
}

static void bs_put_ue(struct bitstream *bs, u32 val)
{
	u32 val_plus_1 = val + 1;
	int zeros = fls(val_plus_1) - 1;
	int i;

	for (i = 0; i < zeros; i++)
		bs_put_bit(bs, 0);

	bs_put_bits(bs, val_plus_1, zeros + 1);
}

static void bs_put_se(struct bitstream *bs, int val)
{
	u32 ue_val;

	if (val <= 0)
		ue_val = -2 * val;
	else
		ue_val = 2 * val - 1;

	bs_put_ue(bs, ue_val);
}

static void bs_put_byte_align(struct bitstream *bs)
{
	bs_put_bit(bs, 1);
	while (bs->bit_pos & 7)
		bs_put_bit(bs, 0);
}

static size_t bs_get_byte_count(struct bitstream *bs)
{
	return (bs->bit_pos + 7) >> 3;
}

/* Write HEVC VPS (Video Parameter Set) */
static size_t hivenc_hevc_build_vps(struct hivenc_ctx *ctx, u8 *buf, size_t max_len)
{
	struct bitstream bs;

	bs_init(&bs, buf, max_len);

	/* Start code */
	bs_put_bits(&bs, 0x00000001, 32);
	/* NAL Header: forbidden(0), type=32(VPS), layer_id=0, tid=1 -> 0x4001 */
	bs_put_bits(&bs, 0x4001, 16);

	bs_put_bits(&bs, 0, 4);  /* vps_video_parameter_set_id = 0 */
	bs_put_bits(&bs, 3, 2);  /* vps_reserved_three_2bits */
	bs_put_bits(&bs, 0, 6);  /* vps_max_layers_minus1 = 0 */
	bs_put_bits(&bs, 0, 3);  /* vps_max_sub_layers_minus1 = 0 */
	bs_put_bit(&bs, 1);      /* vps_temporal_id_nesting_flag = 1 */
	bs_put_bits(&bs, 0xffff, 16); /* vps_reserved_0xffff_16bits */

	/* Profile tier level */
	bs_put_bits(&bs, 0, 2);  /* general_profile_space = 0 */
	bs_put_bit(&bs, 0);      /* general_tier_flag = 0 */
	bs_put_bits(&bs, 1, 5);  /* general_profile_idc = 1 (Main Profile) */
	bs_put_bits(&bs, 0x60000000, 32); /* general_profile_compatibility_flag */
	bs_put_bit(&bs, 1);      /* general_progressive_source_flag */
	bs_put_bit(&bs, 0);      /* general_interlaced_source_flag */
	bs_put_bit(&bs, 0);      /* general_non_packed_constraint_flag */
	bs_put_bit(&bs, 1);      /* general_frame_only_constraint_flag */
	bs_put_bits(&bs, 0, 32); /* reserved flags */
	bs_put_bits(&bs, 0, 12);
	bs_put_bits(&bs, 120, 8);/* general_level_idc = 120 (Level 4.0) */

	bs_put_bit(&bs, 1);      /* vps_sub_layer_ordering_info_present_flag = 1 */
	bs_put_ue(&bs, 1);       /* vps_max_dec_pic_buffering_minus1[0] = 1 */
	bs_put_ue(&bs, 0);       /* vps_max_num_reorder_pics[0] = 0 */
	bs_put_ue(&bs, 0);       /* vps_max_latency_increase_plus1[0] = 0 */

	bs_put_bits(&bs, 0, 6);  /* vps_max_layer_id = 0 */
	bs_put_ue(&bs, 0);       /* vps_num_layer_sets_minus1 = 0 */
	bs_put_bit(&bs, 0);      /* vps_timing_info_present_flag = 0 */
	bs_put_bit(&bs, 0);      /* vps_extension_flag = 0 */

	bs_put_byte_align(&bs);

	return bs_get_byte_count(&bs);
}

/* Write HEVC SPS */
static size_t hivenc_hevc_build_sps(struct hivenc_ctx *ctx, u8 *buf, size_t max_len)
{
	struct bitstream bs;
	u32 width = ctx->out_q.width;
	u32 height = ctx->out_q.height;
	u32 ctu_size = 64;
	u32 min_cb_size = 8;
	u32 crop_right = ((width + ctu_size - 1) / ctu_size * ctu_size) - width;
	u32 crop_bottom = ((height + ctu_size - 1) / ctu_size * ctu_size) - height;

	bs_init(&bs, buf, max_len);

	/* Start code */
	bs_put_bits(&bs, 0x00000001, 32);
	/* NAL Header: type=33(SPS) -> 0x4201 */
	bs_put_bits(&bs, 0x4201, 16);

	bs_put_bits(&bs, 0, 4);  /* sps_video_parameter_set_id = 0 */
	bs_put_bits(&bs, 0, 3);  /* sps_max_sub_layers_minus1 = 0 */
	bs_put_bit(&bs, 1);      /* sps_temporal_id_nesting_flag = 1 */

	/* Profile tier level */
	bs_put_bits(&bs, 0, 2);  /* general_profile_space = 0 */
	bs_put_bit(&bs, 0);      /* general_tier_flag = 0 */
	bs_put_bits(&bs, 1, 5);  /* general_profile_idc = 1 (Main Profile) */
	bs_put_bits(&bs, 0x60000000, 32);
	bs_put_bit(&bs, 1);
	bs_put_bit(&bs, 0);
	bs_put_bit(&bs, 0);
	bs_put_bit(&bs, 1);
	bs_put_bits(&bs, 0, 32);
	bs_put_bits(&bs, 0, 12);
	bs_put_bits(&bs, 120, 8);/* general_level_idc = 120 */

	bs_put_ue(&bs, 0);       /* sps_seq_parameter_set_id = 0 */
	bs_put_ue(&bs, 1);       /* chroma_format_idc: 1 = 4:2:0 */
	bs_put_ue(&bs, width);   /* pic_width_in_luma_samples */
	bs_put_ue(&bs, height);  /* pic_height_in_luma_samples */

	if (crop_right || crop_bottom) {
		bs_put_bit(&bs, 1); /* conformance_window_flag = 1 */
		bs_put_ue(&bs, 0);
		bs_put_ue(&bs, crop_right / 2);
		bs_put_ue(&bs, 0);
		bs_put_ue(&bs, crop_bottom / 2);
	} else {
		bs_put_bit(&bs, 0); /* conformance_window_flag = 0 */
	}

	bs_put_ue(&bs, 0);       /* bit_depth_luma_minus8 = 0 */
	bs_put_ue(&bs, 0);       /* bit_depth_chroma_minus8 = 0 */
	bs_put_ue(&bs, 4);       /* log2_max_pic_order_cnt_lsb_minus4 = 4 (8 bits) */

	bs_put_bit(&bs, 1);      /* sps_sub_layer_ordering_info_present_flag = 1 */
	bs_put_ue(&bs, 1);       /* sps_max_dec_pic_buffering_minus1[0] = 1 */
	bs_put_ue(&bs, 0);       /* sps_max_num_reorder_pics[0] = 0 */
	bs_put_ue(&bs, 0);       /* sps_max_latency_increase_plus1[0] = 0 */

	/* CTU 64, Min CB 8 -> log2_min_luma_coding_block_size_minus3 = 0, diff = 3 */
	bs_put_ue(&bs, ilog2(min_cb_size) - 3); /* log2_min_luma_coding_block_size_minus3 */
	bs_put_ue(&bs, ilog2(ctu_size) - ilog2(min_cb_size)); /* log2_diff_max_min_luma_coding_block_size */
	bs_put_ue(&bs, 0);       /* log2_min_luma_transform_block_size_minus2 = 0 (4x4) */
	bs_put_ue(&bs, 3);       /* log2_diff_max_min_luma_transform_block_size = 3 (32x32) */
	bs_put_ue(&bs, 1);       /* max_transform_hierarchy_depth_inter = 1 */
	bs_put_ue(&bs, 1);       /* max_transform_hierarchy_depth_intra = 1 */

	bs_put_bit(&bs, 0);      /* scaling_list_enabled_flag = 0 */
	bs_put_bit(&bs, 0);      /* amp_enabled_flag = 0 */
	bs_put_bit(&bs, 1);      /* sample_adaptive_offset_enabled_flag = 1 (SAO) */
	bs_put_bit(&bs, 0);      /* pcm_enabled_flag = 0 */

	bs_put_ue(&bs, 0);       /* num_short_term_ref_pic_sets = 0 */
	bs_put_bit(&bs, 0);      /* long_term_ref_pics_present_flag = 0 */
	bs_put_bit(&bs, 0);      /* sps_temporal_mvp_enabled_flag = 0 */
	bs_put_bit(&bs, 1);      /* strong_intra_smoothing_enabled_flag = 1 */

	bs_put_bit(&bs, 0);      /* vui_parameters_present_flag = 0 */
	bs_put_bit(&bs, 0);      /* sps_extension_present_flag = 0 */

	bs_put_byte_align(&bs);

	return bs_get_byte_count(&bs);
}

/* Write HEVC PPS */
static size_t hivenc_hevc_build_pps(struct hivenc_ctx *ctx, u8 *buf, size_t max_len)
{
	struct bitstream bs;

	bs_init(&bs, buf, max_len);

	/* Start code */
	bs_put_bits(&bs, 0x00000001, 32);
	/* NAL Header: type=34(PPS) -> 0x4401 */
	bs_put_bits(&bs, 0x4401, 16);

	bs_put_ue(&bs, 0);       /* pps_pic_parameter_set_id = 0 */
	bs_put_ue(&bs, 0);       /* pps_seq_parameter_set_id = 0 */
	bs_put_bit(&bs, 0);      /* dependent_slice_segments_enabled_flag = 0 */
	bs_put_bit(&bs, 0);      /* output_flag_present_flag = 0 */
	bs_put_bits(&bs, 0, 3);  /* num_extra_slice_header_bits = 0 */
	bs_put_bit(&bs, 0);      /* sign_data_hiding_enabled_flag = 0 */
	bs_put_bit(&bs, 0);      /* cabac_init_present_flag = 0 */
	bs_put_ue(&bs, 0);       /* num_ref_idx_l0_default_active_minus1 = 0 */
	bs_put_ue(&bs, 0);       /* num_ref_idx_l1_default_active_minus1 = 0 */
	bs_put_se(&bs, 0);       /* init_qp_minus26 = 0 */
	bs_put_bit(&bs, 0);      /* constrained_intra_pred_flag = 0 */
	bs_put_bit(&bs, 0);      /* transform_skip_enabled_flag = 0 */
	bs_put_bit(&bs, 1);      /* cu_qp_delta_enabled_flag = 1 */
	bs_put_ue(&bs, 0);       /* diff_cu_qp_delta_depth = 0 */
	bs_put_se(&bs, 0);       /* pps_cb_qp_offset = 0 */
	bs_put_se(&bs, 0);       /* pps_cr_qp_offset = 0 */
	bs_put_bit(&bs, 0);      /* pps_slice_chroma_qp_offsets_present_flag = 0 */
	bs_put_bit(&bs, 0);      /* weighted_pred_flag = 0 */
	bs_put_bit(&bs, 0);      /* weighted_bipred_flag = 0 */
	bs_put_bit(&bs, 0);      /* transquant_bypass_enabled_flag = 0 */
	bs_put_bit(&bs, 0);      /* tiles_enabled_flag = 0 */
	bs_put_bit(&bs, 0);      /* entropy_coding_sync_enabled_flag = 0 */
	bs_put_bit(&bs, 0);      /* loop_filter_across_slices_enabled_flag = 0 */
	bs_put_bit(&bs, 1);      /* deblocking_filter_control_present_flag = 1 */
	bs_put_bit(&bs, 0);      /* deblocking_filter_override_enabled_flag = 0 */
	bs_put_bit(&bs, 0);      /* pps_deblocking_filter_disabled_flag = 0 */
	bs_put_se(&bs, 0);       /* pps_beta_offset_div2 = 0 */
	bs_put_se(&bs, 0);       /* pps_tc_offset_div2 = 0 */
	bs_put_bit(&bs, 0);      /* pps_scaling_list_data_present_flag = 0 */
	bs_put_bit(&bs, 0);      /* lists_modification_present_flag = 0 */
	bs_put_ue(&bs, 0);       /* log2_parallel_merge_level_minus2 = 0 */
	bs_put_bit(&bs, 0);      /* slice_segment_header_extension_present_flag = 0 */
	bs_put_bit(&bs, 0);      /* pps_extension_present_flag = 0 */

	bs_put_byte_align(&bs);

	return bs_get_byte_count(&bs);
}

int hivenc_hevc_write_headers(struct hivenc_ctx *ctx, u8 *buf, size_t max_len)
{
	size_t vps_len = hivenc_hevc_build_vps(ctx, buf, max_len);
	size_t sps_len = hivenc_hevc_build_sps(ctx, buf + vps_len, max_len - vps_len);
	size_t pps_len = hivenc_hevc_build_pps(ctx, buf + vps_len + sps_len,
					       max_len - vps_len - sps_len);

	return vps_len + sps_len + pps_len;
}

void hivenc_hevc_setup_frame(struct hivenc_ctx *ctx, bool is_idr)
{
	struct hivenc_dev *venc = ctx->dev;
	struct bitstream bs;
	u8 slc_hdr_buf[32];
	u32 nal_hdr;
	int i;

	/*
	 * HEVC NAL Header (16 bits):
	 * IDR_W_RADL (type 19) -> 0x2601
	 * TRAIL_R (type 1)     -> 0x0201
	 */
	nal_hdr = is_idr ? 0x2601 : 0x0201;
	vedu_write(venc, VEDU_CABAC_GLB_CFG, (nal_hdr << 16) | 0x0300);

	/* Build HEVC slice header */
	bs_init(&bs, slc_hdr_buf, sizeof(slc_hdr_buf));
	bs_put_bit(&bs, 1);      /* first_slice_segment_in_pic_flag = 1 */
	if (is_idr)
		bs_put_bit(&bs, 0); /* no_output_of_prior_pics_flag = 0 */
	bs_put_ue(&bs, 0);       /* slice_pic_parameter_set_id = 0 */
	bs_put_ue(&bs, is_idr ? 2 : 1); /* slice_type: 2=I, 1=P */

	bs_put_bits(&bs, ctx->frame_count & 0xff, 8); /* slice_pic_order_cnt_lsb */

	if (!is_idr) {
		bs_put_bit(&bs, 0); /* short_term_ref_pic_set_sps_flag = 0 */
		/* short-term ref pic set */
		bs_put_ue(&bs, 0);  /* num_negative_pics = 0 */
		bs_put_ue(&bs, 1);  /* num_positive_pics = 1 */
		bs_put_ue(&bs, 0);  /* delta_poc_s0_minus1[0] = 0 */
		bs_put_bit(&bs, 1); /* used_by_curr_pic_s0_flag[0] = 1 */
	}

	/* SAO enabled flags */
	bs_put_bit(&bs, 1);      /* slice_sao_luma_flag = 1 */
	bs_put_bit(&bs, 1);      /* slice_sao_chroma_flag = 1 */

	if (!is_idr)
		bs_put_bit(&bs, 0); /* num_ref_idx_active_override_flag = 0 */

	bs_put_se(&bs, 0);       /* slice_qp_delta = 0 */
	bs_put_byte_align(&bs);

	/* Set slice header length */
	vedu_write(venc, VEDU_CABAC_SLCHDR_SIZE, bs.bit_pos & 0x1ff);

	/* Pack into CABAC_SLCHDR_PART2 segments */
	for (i = 0; i < 8; i++) {
		size_t offset = i * 4;
		u32 word = 0;

		if (offset < sizeof(slc_hdr_buf))
			word = (slc_hdr_buf[offset] << 24) |
			       (slc_hdr_buf[offset + 1] << 16) |
			       (slc_hdr_buf[offset + 2] << 8) |
			       slc_hdr_buf[offset + 3];
		vedu_write(venc, VEDU_CABAC_SLCHDR_PART2(i), word);
	}
}
