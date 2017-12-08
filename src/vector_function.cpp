#include"img_prc_cls.h"
//memory
	void ImageProcesser::reserve_vectors(void){
		pts.reserve(point_size);   //特徴点
		npts.reserve(point_size);  //移動後の特徴点
		sts.reserve(point_size);
		ers.reserve(point_size);
//-----特徴点抽出用変数-----
		kpts_rect_p.reserve(clp_point_size);
		kpts_p.reserve(point_size);
		kpts_n.reserve(point_size);
		kpts_rect_n.reserve(clp_point_size);
		descriptor_p.reserve(point_size);
		descriptor_n.reserve(point_size);
		match.reserve(point_size);
		match12.reserve(point_size);
		match21.reserve(point_size);
//Provisional vector z
		ppz.reserve(point_size);
		pnz.reserve(point_size);
		points.reserve(point_size);
		newpoints.reserve(point_size);
		kpoints.reserve(point_size);
		newkpoints.reserve(point_size);
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
	}
	void ImageProcesser::clear_vectors(void){
//		pts.clear();   //特徴点
		npts.clear();  //移動後の特徴点
		sts.clear();
		ers.clear();
//-----特徴点抽出用変数-----
//		keypoints.clear();
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				cp[i][j].clear();
			}
		}	
		jnpts.clear();

	}
	void ImageProcesser::renew_vectors(void){
//		pts.clear();
		ppz.clear();
//		pts.insert(pts.end(),newpoints.begin(),newpoints.end());
		ppz.insert(ppz.end(),nz.begin(),nz.end());
		kpts_p.clear();
		kpts_p.insert(kpts_p.end(),newkpoints.begin(),newkpoints.end());
	}
	void ImageProcesser::clear_dtctvectors(void){
		points.clear();
		newpoints.clear();
		jnewpoints.clear();
		kpoints.clear();
		newkpoints.clear();
		kpts_n.clear();
		z.clear();	
		nz.clear();
		pnz.clear();
		match.clear();
		match12.clear();
		match21.clear();
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				cpt[i][j].clear();
				cnpt[i][j].clear();
				cz[i][j].clear();
			}
		}
		mv_area.clear();
		opt.clear();
	}


