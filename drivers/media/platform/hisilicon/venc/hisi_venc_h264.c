// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon Kirin 990 video encoder: H.264 codec support
 *
 * Implements H.264 SPS/PPS generation and hardware slice header configuration.
 */

#include <linux/kernel.h>
#include <linux/string.h>

#include "hisi_venc.h"

/* Simple bitstream writer */
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

/* Write H.264 SPS */
static size_t hivenc_h264_build_sps(struct hivenc_ctx *ctx, u8 *buf, size_t max_len)
{
	struct bitstream bs;
	u32 width = ctx->out_q.width;
	u32 height = ctx->out_q.height;
	u32 mb_width = (width + 15) / 16;
	u32 mb_height = (height + 15) / 16;
	u32 crop_right = (mb_width * 16) - width;
	u32 crop_bottom = (mb_height * 16) - height;
	u8 profile_idc = 100; /* High Profile */
	u8 level_idc = 41;    /* Level 4.1 */

	if (width <= 1280 && height <= 720)
		level_idc = 31;
	else if (width <= 1920 && height <= 1080)
		level_idc = 41;
	else
		level_idc = 51;

	bs_init(&bs, buf, max_len);

	/* Start code 0x00000001 */
	bs_put_bits(&bs, 0x00000001, 32);
	/* NAL header: forbidden(0) | ref_idc(3) | type(7) = 0x67 */
	bs_put_bits(&bs, 0x67, 8);

	bs_put_bits(&bs, profile_idc, 8);
	bs_put_bits(&bs, 0, 8); /* constraint flags */
	bs_put_bits(&bs, level_idc, 8);
	bs_put_ue(&bs, 0);      /* seq_parameter_set_id = 0 */

	if (profile_idc == 100) {
		bs_put_ue(&bs, 1); /* chroma_format_idc: 1 = 4:2:0 */
		bs_put_ue(&bs, 0); /* bit_depth_luma_minus8 = 0 */
		bs_put_ue(&bs, 0); /* bit_depth_chroma_minus8 = 0 */
		bs_put_bit(&bs, 0);/* qpprime_y_zero_transform_bypass_flag */
		bs_put_bit(&bs, 0);/* seq_scaling_matrix_present_flag */
	}

	bs_put_ue(&bs, 0); /* log2_max_frame_num_minus4 = 0 (max 16 frames) */
	bs_put_ue(&bs, 0); /* pic_order_cnt_type = 0 */
	bs_put_ue(&bs, 0); /* log2_max_pic_order_cnt_lsb_minus4 = 0 (max 16) */

	bs_put_ue(&bs, 1); /* max_num_ref_frames = 1 */
	bs_put_bit(&bs, 0);/* gaps_in_frame_num_value_allowed_flag = 0 */

	bs_put_ue(&bs, mb_width - 1);       /* pic_width_in_mbs_minus1 */
	bs_put_ue(&bs, mb_height - 1);      /* pic_height_in_map_units_minus1 */
	bs_put_bit(&bs, 1);                 /* frame_mbs_only_flag = 1 */
	bs_put_bit(&bs, 1);                 /* direct_8x8_inference_flag = 1 */

	if (crop_right || crop_bottom) {
		bs_put_bit(&bs, 1);         /* frame_cropping_flag = 1 */
		bs_put_ue(&bs, 0);          /* frame_crop_left_offset = 0 */
		bs_put_ue(&bs, crop_right / 2); /* crop_right / 2 for 4:2:0 */
		bs_put_ue(&bs, 0);          /* frame_crop_top_offset = 0 */
		bs_put_ue(&bs, crop_bottom / 2);
	} else {
		bs_put_bit(&bs, 0);         /* frame_cropping_flag = 0 */
	}

	bs_put_bit(&bs, 0); /* vui_parameters_present_flag = 0 */
	bs_put_byte_align(&bs);

	return bs_get_byte_count(&bs);
}

/* Write H.264 PPS */
static size_t hivenc_h264_build_pps(struct hivenc_ctx *ctx, u8 *buf, size_t max_len)
{
	struct bitstream bs;

	bs_init(&bs, buf, max_len);

	/* Start code 0x00000001 */
	bs_put_bits(&bs, 0x00000001, 32);
	/* NAL header: forbidden(0) | ref_idc(3) | type(8) = 0x68 */
	bs_put_bits(&bs, 0x68, 8);

	bs_put_ue(&bs, 0); /* pic_parameter_set_id = 0 */
	bs_put_ue(&bs, 0); /* seq_parameter_set_id = 0 */
	bs_put_bit(&bs, 1);/* entropy_coding_mode_flag: 1 = CABAC */
	bs_put_bit(&bs, 0);/* bottom_field_pic_order_in_frame_present_flag = 0 */
	bs_put_ue(&bs, 0); /* num_slice_groups_minus1 = 0 */
	bs_put_ue(&bs, 0); /* num_ref_idx_l0_default_active_minus1 = 0 */
	bs_put_ue(&bs, 0); /* num_ref_idx_l1_default_active_minus1 = 0 */
	bs_put_bit(&bs, 0);/* weighted_pred_flag = 0 */
	bs_put_bits(&bs, 0, 2); /* weighted_bipred_idc = 0 */
	bs_put_se(&bs, 0); /* pic_init_qp_minus26 = 0 */
	bs_put_se(&bs, 0); /* pic_init_qs_minus26 = 0 */
	bs_put_se(&bs, 0); /* chroma_qp_index_offset = 0 */
	bs_put_bit(&bs, 1);/* deblocking_filter_control_present_flag = 1 */
	bs_put_bit(&bs, 0);/* constrained_intra_pred_flag = 0 */
	bs_put_bit(&bs, 0);/* redundant_pic_cnt_present_flag = 0 */
	bs_put_bit(&bs, 1);/* transform_8x8_mode_flag = 1 */
	bs_put_bit(&bs, 0);/* pic_scaling_matrix_present_flag = 0 */
	bs_put_se(&bs, 0); /* second_chroma_qp_index_offset = 0 */

	bs_put_byte_align(&bs);

	return bs_get_byte_count(&bs);
}

int hivenc_h264_write_headers(struct hivenc_ctx *ctx, u8 *buf, size_t max_len)
{
	size_t sps_len = hivenc_h264_build_sps(ctx, buf, max_len);
	size_t pps_len = hivenc_h264_build_pps(ctx, buf + sps_len, max_len - sps_len);

	return sps_len + pps_len;
}

void hivenc_h264_setup_frame(struct hivenc_ctx *ctx, bool is_idr)
{
	struct hivenc_dev *venc = ctx->dev;
	struct bitstream bs;
	u8 slc_hdr_buf[16];
	u32 slc_word0, slc_word1;
	u32 nal_hdr;

	/* Setup NAL header: IDR=0x65, Non-IDR P=0x41 */
	nal_hdr = is_idr ? 0x65 : 0x41;
	vedu_write(venc, VEDU_CABAC_GLB_CFG, (nal_hdr << 16) | 0x0300);

	/* Build slice header bitstream */
	bs_init(&bs, slc_hdr_buf, sizeof(slc_hdr_buf));
	bs_put_ue(&bs, 0); /* first_mb_in_slice = 0 */
	bs_put_ue(&bs, is_idr ? 7 : 5); /* slice_type: 7=I_ALL (or 2=I), 5=P_ALL (or 0=P) */
	bs_put_ue(&bs, 0); /* pic_parameter_set_id = 0 */
	bs_put_bits(&bs, ctx->frame_count & 0xf, 4); /* frame_num (4 bits) */

	if (is_idr) {
		bs_put_ue(&bs, 0); /* idr_pic_id = 0 */
	}

	bs_put_bits(&bs, (ctx->frame_count * 2) & 0xf, 4); /* pic_order_cnt_lsb (4 bits) */

	if (!is_idr) {
		bs_put_bit(&bs, 0); /* num_ref_idx_active_override_flag = 0 */
		bs_put_bit(&bs, 0); /* ref_pic_list_modification_flag_l0 = 0 */
	}

	/* dec_ref_pic_marking */
	if (is_idr) {
		bs_put_bit(&bs, 0); /* no_output_of_prior_pics_flag = 0 */
		bs_put_bit(&bs, 0); /* long_term_reference_flag = 0 */
	} else {
		bs_put_bit(&bs, 0); /* adaptive_ref_pic_marking_mode_flag = 0 */
	}

	bs_put_se(&bs, 0); /* slice_qp_delta = 0 */
	/* deblocking filter */
	bs_put_ue(&bs, 0); /* disable_deblocking_filter_idc = 0 */
	bs_put_se(&bs, 0); /* slice_alpha_c0_offset_div2 = 0 */
	bs_put_se(&bs, 0); /* slice_beta_offset_div2 = 0 */

	/* Pack header stream words into hardware registers */
	slc_word0 = (slc_hdr_buf[0] << 24) | (slc_hdr_buf[1] << 16) |
		    (slc_hdr_buf[2] << 8) | slc_hdr_buf[3];
	slc_word1 = (slc_hdr_buf[4] << 24) | (slc_hdr_buf[5] << 16) |
		    (slc_hdr_buf[6] << 8) | slc_hdr_buf[7];

	vedu_write(venc, VEDU_VLC_SLCHDRSTRM(0), slc_word0);
	vedu_write(venc, VEDU_VLC_SLCHDRSTRM(1), slc_word1);
	vedu_write(venc, VEDU_VLC_SLCHDRSTRM(2), 0);
	vedu_write(venc, VEDU_VLC_SLCHDRSTRM(3), 0);

	/* Set slice header bit length (vlc_parabit) */
	vedu_write(venc, VEDU_VLC_SLCHDRPARA, (bs.bit_pos & 0x7f) << 16);
}
