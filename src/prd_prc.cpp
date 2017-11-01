#include"img_prc_cls.h"

	void ImageProcesser::prd_prcess(void){
		//方針
		//分割領域のオプティカルフローの大きさの平均or平均と分散を算出
		//オプティカルフローがない領域なども考慮に入れること
		//領域全体の平均と分散から、平均からより離れている領域に番号付け（ソート）を行い、移動物体領域を検出
		//デバック::より移動物体として認識されている領域に色を加える
		
		//分割画像の各特徴点の平均と分散(x,y,size)
//		std::vector<cv::Point2d> cpt[cn][cn];//img_prc_cls.h
		double sumpt[cn][cn];
		for(int i=0;i<cn;i++)
			for(int j=0;j<cn;j++)
				sumpt[i][j]=0;
		for(int k=0;k<points.size();k++){
			for(int j=0;j<cn;j++){
				if((int)(j*width/cn) < (int)points[k].x && (int)points[k].x < (int)((j+1)*width/cn)){
					for(int i=0;i<cn;i++){
						if((int)(i*height/cn)<(int)points[k].y&&(int)points[k].y<(int)((i+1)*height/cn)){

		cpt[i][j].push_back(pts[k]);

						}
					}
				}
			}
		}
		
		

		
	}

