#include"img_prc_cls.h"
//memory
	void ImageProcesser::reserve_vectors(void){
		pts.reserve(point_size);   //特徴点
		npts.reserve(point_size);  //移動後の特徴点
		sts.reserve(point_size);
		ers.reserve(point_size);
//-----特徴点抽出用変数-----
		keypoints.reserve(point_size);
//Provisional vector z
		pz.reserve(point_size);
		points.reserve(point_size);
		newpoints.reserve(point_size);
		z.reserve(point_size);
		nz.reserve(point_size);
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				cp[i][j].reserve(clp_point_size);
				cpt[i][j].reserve(clp_point_size);
				cnpt[i][j].reserve(clp_point_size);
				cz[i][j].reserve(clp_point_size);
				cpt_num[i][j].reserve(clp_point_size);
			}
		}
		jnpts.reserve(point_size);
		jnewpoints.reserve(point_size);
		mv_area.reserve(cnh*cnw);
		opt.reserve(cnh*cnw);
		tracking_count_p.reserve(point_size);
		tracking_count.reserve(point_size);

		space_begin.reserve(cnw);
		space_end.reserve(cnw);
		space_size.reserve(cnw);		
		area_begin.reserve(cnw*cnh);
		area_end.reserve(cnw*cnh);
		area_opt.reserve(point_size);
		area_z.reserve(cnw*cnh);
		is_stc_pts.reserve(point_size);
		is_stc_pts_p.reserve(point_size);
		is_mv_pts.reserve(point_size);
		depth_median.reserve(ksize*ksize);
	}
	void ImageProcesser::clear_vectors(void){
		npts.clear();  //移動後の特徴点
		sts.clear();
		ers.clear();
//-----特徴点抽出用変数-----
		keypoints.clear();
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				cp[i][j].clear();
			}
		}	
		jnpts.clear();
	}
	void ImageProcesser::renew_vectors(void){
		pts.clear();
		pz.clear();
		pts.insert(pts.end(),newpoints.begin(),newpoints.end());
		pz.insert(pz.end(),nz.begin(),nz.end());
		tracking_count_p.clear();
		tracking_count_p.insert(tracking_count_p.end(),tracking_count.begin(),tracking_count.end());
		tracking_count.clear();	
	}
	void ImageProcesser::clear_dtctvectors(void){
		points.clear();
		newpoints.clear();
		jnewpoints.clear();
		z.clear();	
		nz.clear();
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				cpt[i][j].clear();
				cpt_num[i][j].clear();
				cnpt[i][j].clear();
				cz[i][j].clear();
			}
		}
		mv_area.clear();
		opt.clear();
		
		space_begin.clear();
		space_end.clear();
		space_size.clear();
/*
		area_begin.clear();
		area_end.clear();
		area_opt.clear();
		area_z.clear();
*/		
		is_stc_pts_p.clear();
		is_stc_pts_p.insert(is_stc_pts_p.end(),is_stc_pts.begin(),is_stc_pts.end());
		is_stc_pts.clear();
		is_mv_pts_p.clear();
		is_mv_pts_p.insert(is_mv_pts_p.end(),is_mv_pts.begin(),is_mv_pts.end());
		is_mv_pts.clear();

	}


