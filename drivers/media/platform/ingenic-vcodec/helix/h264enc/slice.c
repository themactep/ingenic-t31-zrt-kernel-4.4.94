#include "cabac.h"
#include "slice.h"


void h264e_slice_header_init(h264_slice_header_t *sh,
				h264_sps_t *sps, h264_pps_t *pps,
				int i_idr_pic_id, int i_frame, int i_qp)
{
	sh->sps = sps;
	sh->pps = pps;

	sh->i_first_mb = 0;
	sh->i_last_mb = sps->i_mb_width * sps->i_mb_height - 1;
	sh->i_pps_id = pps->i_id;
	sh->i_frame_num = i_frame;

	sh->b_mbaff = 0;
	sh->b_field_pic = 0;
	sh->b_bottom_field = 0;

	sh->i_idr_pic_id = i_idr_pic_id;

	/* poc stuff, fixed later */
	/* vpu用不着 ..., 固定顺序编码 ...*/
	sh->i_poc = 0;
	sh->i_delta_poc_bottom = 0;
	sh->i_delta_poc[0] = 0;
	sh->i_delta_poc[1] = 0;


	/*VPU 当前硬件用不着.*/
	sh->i_redundant_pic_cnt = 0;

	/*也用不着，写固定值就好了.*/
	sh->b_num_ref_idx_override = 0;
	sh->i_num_ref_idx_l0_active = 1;
	sh->i_num_ref_idx_l1_active = 1;

	/*不做重排序了，重排序有什么好处?，软件去做的.*/
	sh->b_ref_pic_list_reordering[0] = 0;
	sh->b_ref_pic_list_reordering[1] = 0;


	/* TODO : value to be */
	sh->i_cabac_init_idc = 0; /* TODO, 是否可以使用该变量来表示初始化cabac_init？？？*/

	sh->i_qp = SPEC_QP(i_qp);
	sh->i_qp_delta = sh->i_qp - pps->i_pic_init_qp;

	sh->b_sp_for_swidth = 0;
	sh->i_qs_delta = 0;


	/*TODO: 是否要关闭 快过滤功能，先关闭，后面再打开. */
	sh->i_disable_deblocking_filter_idc = 1;
	sh->i_alpha_c0_offset = 0;
	sh->i_beta_offset = 0;

}


void h264e_slice_header_write(bs_t *s, h264_slice_header_t *sh, int i_nal_ref_idc)
{
	/*Do not support mbaff*/
	bs_write_ue(s, sh->i_first_mb);
	bs_write_ue(s, sh->i_type + 5);
	bs_write_ue(s, sh->i_pps_id);
	bs_write(s, sh->sps->i_log2_max_frame_num, sh->i_frame_num & ((1 << sh->sps->i_log2_max_frame_num) - 1));

	/*mbs only*/

	if(sh->i_idr_pic_id >= 0)
		bs_write_ue(s, sh->i_idr_pic_id);

#if 0
	/* poc_type 只支持 2. */
	if(sh->sps->i_poc_type == 0) {
		bs_write(s, sh->sps->i_log2_max_poc_lsb, sh->i_poc & ((1 << sh->sps->i_log2_max_poc_lsb) - 1));
		if(sh->pps->b_pic_order && !sh->b_field_pic)
			bs_write_se(s, sh->i_delta_poc_bottom);
	}
#endif

#if 0
	/*main profile 0就是0，不改了*/
	if(sh->pps->b_redundant_pic_cnt)
		bs_write_ue(s, sh->i_redundant_pic_cnt);
#endif
#if 0
	if(sh->i_type == SLICE_TYPE_B) {
		/*not support*/
		bs_write1(s, sh->b_direct_spatial_mv_pred);
	}
#endif

	if(sh->i_type == SLICE_TYPE_P || sh->i_type == SLICE_TYPE_B) {
		bs_write1(s, sh->b_num_ref_idx_override);
		if(sh->b_num_ref_idx_override) {
			bs_write_ue(s, sh->i_num_ref_idx_l0_active - 1);
			if(sh->i_type == SLICE_TYPE_B)
				bs_write_ue(s, sh->i_num_ref_idx_l1_active - 1);
		}

	}

	/* 不reorder了。。, 初始化为0就好了. */
	if(sh->i_type != SLICE_TYPE_I) {
		bs_write1(s, sh->b_ref_pic_list_reordering[0]);
		if(sh->b_ref_pic_list_reordering[0]) {
			int i;
			for(i = 0; i < sh->i_num_ref_idx_l0_active; i++) {
				bs_write_ue(s, sh->ref_pic_list_order[0][i].idc);
				bs_write_ue(s, sh->ref_pic_list_order[0][i].arg);
			}

			bs_write_ue(s, 3);
		}
	}

#if 0
	/*不支持 SLICE_TYPE_B */
	if(sh->i_type == SLICE_TYPE_B) {
		/*not support, IGNORE...*/
	}
#endif
#if 0
	/*使用默认的加权预测方式*/
	sh->b_weighted_pred = 0;
	if(sh->pps->b_weighted_pred && sh->i_type == SLICE_TYPE_P) {
		/*TODO, 暂时先不管预测 */
	}
#endif

	if(i_nal_ref_idc != 0) {
		if(sh->i_idr_pic_id >= 0) {
			bs_write1(s, 0);
			bs_write1(s, 0);
		} else {
			bs_write1( s, sh->i_mmco_command_count > 0 ); /* adaptive_ref_pic_marking_mode_flag */
			if( sh->i_mmco_command_count > 0 )
			{
				int i = 0;
				for(i = 0; i < sh->i_mmco_command_count; i++ )
				{
					bs_write_ue( s, 1 ); /* mark short term ref as unused */
					bs_write_ue( s, sh->mmco[i].i_difference_of_pic_nums - 1 );
				}
				bs_write_ue( s, 0 ); /* end command list */
			}

		}

	}

	if( sh->pps->b_cabac && sh->i_type != SLICE_TYPE_I )
		bs_write_ue( s, sh->i_cabac_init_idc );

	bs_write_se( s, sh->i_qp_delta );      /* slice qp delta */

	if( sh->pps->b_deblocking_filter_control )
	{
		bs_write_ue( s, sh->i_disable_deblocking_filter_idc );
		if( sh->i_disable_deblocking_filter_idc != 1 )
		{
			bs_write_se( s, sh->i_alpha_c0_offset >> 1 );
			bs_write_se( s, sh->i_beta_offset >> 1 );
		}
	}
}
