#include"img_prc_cls.h"

bool ImageProcesser::judge_feature_points(void){
  
  if(threshold_fp>(int)pts.size())
    return true;
  else
    return false;
}

void ImageProcesser::add_feature_points(void){
//	std::cout<<"add_fp\n";
//	auto detector = cv::ORB(max_points, 1.25f, 4, 7, 0, 2, 0, 7);
//	detector.detect(PreLgray, keypoints);
//	std::cout<<"keypoints size:"<<keypoints.size()<<"\n";
	float ptz;
	bool flag;
	for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
		 itk != keypoints.end(); ++itk){
		   flag=false;
		   for(int i=0;i<pts.size();i++){
		     if((int)pts[i].x==(int)itk->pt.x&&(int)pts[i].y==(int)itk->pt.y){
			std::cout<<"points[i].x==itk->pt.x&&points[i].y==itk->pt.y::"<<pts[i].x<<"=="<<itk->pt.x<<"&&"<<pts[i].y<<"=="<<itk->pt.y<<"\n";
		       flag=true;
		       break;
		     }
		   }
		   if(flag)
		     continue;
		
		   ptz=depth_img.at<float>(
		        itk->pt.y,
		        itk->pt.x
		        );
		if(!std::isnan(ptz)){
		   	pts.push_back(itk->pt);
			pz.push_back(ptz);
		}
	}
}
  
  
