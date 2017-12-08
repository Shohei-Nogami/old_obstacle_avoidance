#include"img_prc_cls.h"

	void ImageProcesser::add_feature_points(void){
		auto orb = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
	//	auto akaze = cv::AKAZE::create();	
	//	cv::Ptr<cv::AKAZE> detector =cv::AKAZE::create();
		cv::Point2i ppts;
		float ptz;
		bool flag;
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				if((int)cp[i][j].size()<th_clpimg){
					orb.detect(clp_img_p[i][j], kpts_rect_p);
					orb.detect(clp_img_n[i][j], kpts_rect_n);
					cv::KeyPoint keypoints_temp;

					//add prvious points
					for(int k=0;k<kpts_rect_p.size();k++){// ++i){
						flag=false;
					 	ppts.x=j*width/cnw+kpts_rect_p[k].pt.x;
						ppts.y=i*height/cnh+kpts_rect_p[k].pt.y;
						keypoints_temp=kpts_rect_p[k];
						keypoints_temp.pt.x=j*width/cnw+kpts_rect_p[k].pt.x;
						keypoints_temp.pt.y=i*height/cnh+kpts_rect_p[k].pt.y;
						for(int l=0;l<pts.size();l++){
							if(std::abs(cp[i][j][l].x-ppts.x)<1&&std::abs(cp[i][j][l].y-ppts.y)<1){
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
							ppz.push_back(ptz);
							kpts_p.push_back(keypoints_temp);
						}
					}
					//add current points
					for(int k=0;k<kpts_rect_p.size();k++){// ++i){
						cv::Point2i pnpts;
					 	pnpts.x=j*width/cnw+kpts_rect_n[k].pt.x;
						pnpts.y=i*height/cnh+kpts_rect_n[k].pt.y;
						keypoints_temp=kpts_rect_n[k];
						keypoints_temp.pt.x=j*width/cnw+kpts_rect_n[k].pt.x;
						keypoints_temp.pt.y=i*height/cnh+kpts_rect_n[k].pt.y;						
						float depth_np=depth_img.at<float>(pnpts.y,pnpts.x);
						if(!std::isnan(depth_np)&&!std::isinf(depth_np)&&depth_np>=0.5){
							pnz.push_back(depth_np);
							kpts_n.push_back(keypoints_temp);
						}
					}
				}
			}
		}
	}
	
	void ImageProcesser::count_feature_points(void){
		//分割画像の各特徴点数を算出
		for(int k=0;k<kpts_p.size();k++){//pts.size();k++){
			for(int j=0;j<cnw;j++){
				if((int)(j*width/cnw) < (int)pts[k].x && (int)pts[k].x < (int)((j+1)*width/cnw)){
					for(int i=0;i<cnh;i++){
						if((int)(i*height/cnh)<(int)pts[k].y&&(int)pts[k].y<(int)((i+1)*height/cnh)){
							cp[i][j].push_back(kpts_p[k].pt);//pts[k]);
						}
					}
				}
			}
		}
	}

