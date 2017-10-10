#include"img_prc_cls.h"
//memory
	void ImageProcesser::reserve_vectors(void){
		int size=8000;
		pts.reserve(size);   //特徴点
		npts.reserve(size);  //移動後の特徴点
		sts.reserve(size);
		ers.reserve(size);
//-----特徴点抽出用変数-----
		keypoints.reserve(size);
//Provisional vector z
		pz.reserve(size);
		points.reserve(size);
		newpoints.reserve(size);
		z.reserve(size);
	}
	void ImageProcesser::clear_vectors(void){
		pts.clear();   //特徴点
		npts.clear();  //移動後の特徴点
		sts.clear();
		ers.clear();
//-----特徴点抽出用変数-----
		keypoints.clear();
//Provisional vector z
		pz.clear();
		points.clear();
		newpoints.clear();
		z.clear();	
	}
