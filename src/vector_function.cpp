#include"img_prc_cls.h"
//memory
	void ImageProcesser::reserve_vectors(void){
//		int size=80;
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
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				cp[i][j].reserve(clp_point_size);
//				cp_s[i][j].reserve(clp_point_size);
			}
		}
	}
	void ImageProcesser::clear_vectors(void){
//		pts.clear();   //特徴点
		npts.clear();  //移動後の特徴点
		sts.clear();
		ers.clear();
//-----特徴点抽出用変数-----
		keypoints.clear();
//Provisional vector z
//		pz.clear();
		points.clear();
		newpoints.clear();
		z.clear();	
		nz.clear();
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				cp[i][j].clear();
//				cp_s[i][j].clear();
			}
		}	
	}
	void ImageProcesser::renew_vectors(void){
		pts.clear();
		pz.clear();
		pts.insert(pts.end(),newpoints.begin(),newpoints.end());
		pz.insert(pz.end(),nz.begin(),nz.end());
		
	}
