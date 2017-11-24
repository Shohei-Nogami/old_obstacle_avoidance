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
			}
		}
		jnpts.reserve(point_size);
		jnewpoints.reserve(point_size);
		mv_area.reserve(cnh*cnw);
		opt.reserve(cnh*cnw);

		sgm.reserve(point_size);
		sgm_p.reserve(point_size);
		xt_hat.reserve(point_size);
		xt_hat_p.reserve(point_size);
		xt_once_p.reserve(point_size);
		xt_once.reserve(point_size);
	}
	void ImageProcesser::clear_vectors(void){
//		pts.clear();   //特徴点
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
		xt_hat_p.clear();
		xt_hat_p.insert(xt_hat_p.end(),xt_hat.begin(),xt_hat.end());
		sgm_p.clear();
		sgm_p.insert(sgm_p.end(),sgm.begin(),sgm.end());
		xt_once_p.clear();
		xt_once_p.insert(xt_once_p.end(),xt_once.begin(),xt_once.end());
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
				cnpt[i][j].clear();
				cz[i][j].clear();
			}
		}
		mv_area.clear();
		opt.clear();

		sgm.clear();
		xt_hat.clear();
		xt_once.clear();
	}


