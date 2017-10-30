#include"img_prc_cls.h"

void ImageProcesser::add_feature_points(void){
//	std::cout<<"add_fp\n";
	auto detector = cv::ORB(clp_max_points, 1.25f, 4, /*7*/31, 0, 2, 0, 31);
//	detector.detect(PreLgray, keypoints);
//	std::cout<<"keypoints size:"<<keypoints.size()<<"\n";
	cv::Point2i ppts;
	float ptz;
	bool flag;
	for(int i=0;i<cn;i++){
		for(int j=0;j<cn;j++){
			if((int)cp[i][j].size()<th_clpimg){
				detector.detect(clp_img[i][j], keypoints);	
				for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
		 			itk != keypoints.end(); ++itk){
					flag=false;
				   	ppts.x=j*width/cn+itk->pt.x;
					ppts.y=i*height/cn+itk->pt.y;
					for(int k=0;k<pts.size();k++){
						if(std::abs(cp[i][j][k].x-ppts.x)<1&&std::abs(cp[i][j][k].y-ppts.y)<1){
						//	std::cout<<"points[i].x==itk->pt.x&&points[i].y==itk->pt.y::"<<pts[i].x<<"=="<<itk->pt.x<<"&&"<<pts[i].y<<"=="<<itk->pt.y<<"\n";
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
						pmpf.push_back(0);
					}
				}
			}
		}
	}
}
  
  void ImageProcesser::count_feature_points(void){
    /*
    //分割画像の各特徴点数を算出
int cp_s[n][n]={0}  //in class
for s s<p_s s++
  for j j<n j++
    if j×width/n<pts[s].x&&pts[s].x<(j+1)×width/n
      for i i<n i++
        if  i×height/n<pts[s]&&pts[s]<(i+1)×height/n
          cp_s[i][j]++
    */
 //   cp_s[cn][cn]={0};//nは任意
       //std::vector<cv::Point2i> cp[n][n];
       /*
       for(int i=0;i<n;i++)
         for(int j=0;j<n;j++)
           cp_s[i][j]=0;
       */

    for(int k=0;k<pts.size();k++){
      for(int j=0;j<cn;j++){
        if((int)(j*width/cn) < (int)pts[k].x && (int)pts[k].x < (int)((j+1)*width/cn)){
          for(int i=0;i<cn;i++){
            if((int)(i*height/cn)<(int)pts[k].y&&(int)pts[k].y<(int)((i+1)*height/cn)){

		cp[i][j].push_back(pts[k]);

            }
          }
        }
      }
    }
//			print_clpsize();
}
