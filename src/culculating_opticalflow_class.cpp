#include"culculate_opticalflow_class.h"
#include"depth_image_class.h"
#include"odometry_class.h"
#include"wheel_odometry_class.h"

culculate_optical_flow::culculate_optical_flow()
  :point_size(max_points*2)
  ,clp_point_size((int)(clp_max_points*10)),threshold_fp((int)(max_points*0.8))
  ,th_clpimg((int)(clp_max_points*0.8))
  {
  pts.reserve(point_size);   //特徴点
  npts.reserve(point_size);  //移動後の特徴点
  sts.reserve(point_size);
  ers.reserve(point_size);
  keypoints.reserve(point_size);
  pre_z.reserve(point_size);
  cur_z.reserve(point_size);
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
  tracking_count_p.reserve(point_size);
  tracking_count.reserve(point_size);

}

culculate_optical_flow::~culculate_optical_flow(){

}
void culculate_optical_flow::set_gray_images(const cv::Mat& pre_img,const cv::Mat& cur_img){//step1
  cv::cvtColor(pre_img,pre_gray_image,CV_BGR2GRAY);
  cv::cvtColor(cur_img,cur_gray_image,CV_BGR2GRAY);

}

void culculate_optical_flow::set_clip_images(const int& nh=cnh,const int& nw=cnw){//step2
  for(int i=0;i<nh;i++){
    for(int j=0;j<nw;j++){
      clp_img[i][j]=pre_gray_image(cv::Rect((int)(j*(width)/cnw),(int)(i*(height)/cnh),(int)(width/cnw),(int)(height/cnh)));
    }
  }
}
bool culculate_optical_flow::obtain_feature_points(const cv::Mat& pre_image,const int& nh=cnh,const int& nw=cnw){
  //nh=nw=16 -> fp.size:987
  int clp_max_points=max_points/(nh*nw);
  auto detector = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
  cv::Point2i ppts;
  float ppre_z;
  for(int i=0;i<nh;i++){
    for(int j=0;j<nw;j++){
      detector.detect(clp_img[i][j], keypoints);
      for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin()
        itk != keypoints.end(); ++itk){
        ppts.x=(j*width/nw+itk->pt.x)/ksize;
        ppts.y=i*height/nh+itk->pt.y/ksize;
        ppre_z=filted_pre_image.at<float>(
          ppts.y,
          ppts.x
          );
        if(!std::isnan(ppre_z)&&!std::isinf(ppre_z)&&ppre_z>=0.5){
          pts.push_back(ppts);
          pre_z.push_back(ppre_z);
        }//end if
      }//end for
    }//end for
  }//end for
  /*
  for(int i=0;i<nh;i++){
    for(int j=0;j<nw;j++){
      detector.detect(clp_img[i][j], keypoints);
      for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin()
        itk != keypoints.end(); ++itk){
        ppts.x=(j*width/nw+itk->pt.x)/ksize;
        ppts.y=i*height/nh+itk->pt.y/ksize;
        ppre_z=filted_pre_image.at<float>(
          ppts.y,
          ppts.x
          );
        if(ppre_z>=0.5){
          pts.push_back(ppts);
          pre_z.push_back(ppre_z);
          break;
        }//end if
      }//end for
    }//end for
  }//end for
  */

  if(!pts.size()){
    return false;
  }
  return true;
}
void culculate_optical_flow::culculating_observed_opticalflow(const int& window_size=13){
  //return true:pts.size()>0,false:pts.size()==0
  cv::calcOpticalFlowPyrLK(PreLgray,Lgray, pts, npts, sts, ers, cv::Size(window_size,window_size), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);

}
void culculate_optical_flow::culculating_moving_objects_opticalflow(const cv::Mat& cur_image){//+V,Ω
  float pcur_z;
  cv::Point2i ppt;
  cv::Point3f dX_element;

  for(int i=0;i<pts.size();i++){
    if(sts[i]){
      pcur_z=cur_image.at<float>(
          npts[i].y,
          npts[i].x
          );
      if(!std::isnan(pcur_z)&&!std::isinf(pcur_z)&&pcur_z>=0.5){

  //画像ヤコビアンではなく並進と回転行列で計算
       ppt.x=pts[j].x- (float)(//wheel only
        w_v*sin(-dyaw)*dt/pz[j]-X/pz[j]*w_v*cos(-dyaw)*dt
        -(f+pow(X,2.0)/f)*dyaw
        );

      ppt.y=pts[j].y-(float)(
          -(Y/pz[j]*w_v*cos(-dyaw)*dt)
          -(X*Y*dyaw/f
          ));

      points.push_back(pts[i]);
      newpoints.push_back(npts[i]-ppt);

      dX_element.x=dx;
      dX_element.y=dy;
      dX_element.z=dz;
      dX.push_back(dX_element);
      //				newpoints.push_back(npts[i]);
      }
    }
  }
}
void culculate_optical_flow::publish_flow_image(const cv::Mat& cur_image){
  view_image=cur_image.clone();
  for(int i=0;i<pts.size();i++){
    ImageProcesser::cvArrow(&Limg_view,
    cv::Point((int)points[j].x,
      (int)points[j].y),
    cv::Point((int)(newpoints[j].x),
      (int)(newpoints[j].y),
    cv::Scalar(0,200,200));//
  }

  //publish_debug_image(view_image);
}
void culculate_optical_flow::clear_vector(void){
  pts.clear(point_size);   //特徴点
  npts.clear(point_size);  //移動後の特徴点
  sts.clear(point_size);
  ers.clear(point_size);
  keypoints.clear(point_size);
  pre_z.clear(point_size);
  cur_z.clear(point_size);
  points.clear(point_size);
  newpoints.clear(point_size);
  z.clear(point_size);
  nz.clear(point_size);
  for(int i=0;i<cnh;i++){
    for(int j=0;j<cnw;j++){
      cp[i][j].clear(clp_point_size);
      cpt[i][j].clear(clp_point_size);
      cnpt[i][j].clear(clp_point_size);
      cz[i][j].clear(clp_point_size);
      cpt_num[i][j].clear(clp_point_size);
    }
  }
  jnpts.clear(point_size);
  jnewpoints.clear(point_size);
  tracking_count_p.clear(point_size);
  tracking_count.clear(point_size);
}
cv::Mat& culculate_optical_flow::get_view_image(void){
  return view_image;
}
void culculate_optical_flow::cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color){
  int thickness=4;
  int lineType=8;
  int shift=0;
  cv::line(*img,pt1,pt2,color,thickness,lineType,shift);
  float vx = (float)(pt2.x - pt1.x);
  float vy = (float)(pt2.y - pt1.y);
  float v = sqrt( vx*vx + vy*vy );
  float ux = vx / v;
  float uy = vy / v;
  //矢印の幅の部分
  float w=5,h=10;
  cv::Point2i ptl,ptr;
  ptl.x = (int)((float)pt2.x - uy*w - ux*h);
  ptl.y = (int)((float)pt2.y + ux*w - uy*h);
  ptr.x = (int)((float)pt2.x + uy*w - ux*h);
  ptr.y = (int)((float)pt2.y - ux*w - uy*h);
  //矢印の先端を描画する
  //--例外処理(!(v==0))
  if(!(v==0)){
      cv::line(*img,pt2,ptl,color,thickness,lineType,shift);
      cv::line(*img,pt2,ptr,color,thickness,lineType,shift);
  }
}



int main(int argc,char **argv){
	ros::init(argc,argv,"culculating_opticalflow_class_test");
  image_class img;
	depth_image_class depth_img;


}
