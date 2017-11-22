#include"img_prc_cls.h"

void ImageProcesser::add_feature_points(void){
	auto detector = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
	cv::Point2i ppts;
	float ptz;
	bool flag;
	for(int i=0;i<cnh;i++){
		for(int j=0;j<cnw;j++){
			if((int)cp[i][j].size()<th_clpimg){
				detector.detect(clp_img[i][j], keypoints);	
				for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
		 			itk != keypoints.end(); ++itk){
					flag=false;
				   	ppts.x=j*width/cnw+itk->pt.x;
					ppts.y=i*height/cnh+itk->pt.y;
					for(int k=0;k<pts.size();k++){
						if(std::abs(cp[i][j][k].x-ppts.x)<1&&std::abs(cp[i][j][k].y-ppts.y)<1){
					    	flag=true;
					    	break;
						}
					}
					if(flag)
						continue;
					ptz=Predepth.at<float>(
						ppts.y,
						ppts.x
						);					
					if(!std::isnan(ptz)&&!std::isinf(ptz)&&ptz>=0.5&&(int)pts.size()<point_size){
						pts.push_back(ppts);
						pz.push_back(ptz);
						double mk_data[3][3]={{5,0,0},{0,5,0},{0,0,5}};
						cv::Mat mk_temp(3,3,CV_64FC1,mk_data);
						mk.push_back(mk_temp);
						double pk_data[3][3]={{0,0,0,},{0,0,0},{0,0,0}};
						cv::Mat pk_temp(3,3,CV_64FC1,pk_data);
						pk.push_back(pk_temp);
					}
				}
			}
		}
	}
}
  
  void ImageProcesser::count_feature_points(void){
    
    //分割画像の各特徴点数を算出
    for(int k=0;k<pts.size();k++){
      for(int j=0;j<cnw;j++){
        if((int)(j*width/cnw) < (int)pts[k].x && (int)pts[k].x < (int)((j+1)*width/cnw)){
          for(int i=0;i<cnh;i++){
            if((int)(i*height/cnh)<(int)pts[k].y&&(int)pts[k].y<(int)((i+1)*height/cnh)){
				cp[i][j].push_back(pts[k]);
            }
          }
        }
      }
    }
}

