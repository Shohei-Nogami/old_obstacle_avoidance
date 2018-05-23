#include"culculating_opticalflow_class.h"
#include"depth_image_class.h"
#include"visual_odometry_class.h"
#include"wheel_odometry_class.h"
#include"time_class.h"

culculate_optical_flow::culculate_optical_flow()
  :point_size(max_points*2)
//  ,clp_point_size((int)(clp_max_points*10)),threshold_fp((int)(max_points*0.8))
//  ,th_clpimg((int)(clp_max_points*0.8))
  {
  pub_vel=nh.advertise<obst_avoid::vel3d>("objects_velocity",1);
  
  
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

  jnpts.reserve(point_size);
  jnewpoints.reserve(point_size);
  tracking_count_p.reserve(point_size);
  tracking_count.reserve(point_size);
	
	vX.vel.reserve(point_size);
	vX.pt.reserve(point_size);
	
}

culculate_optical_flow::~culculate_optical_flow(){

}
void culculate_optical_flow::set_gray_images(const cv::Mat& pre_img,const cv::Mat& cur_img){//step1
  cv::cvtColor(pre_img,pre_gray_image,CV_BGR2GRAY);
  cv::cvtColor(cur_img,cur_gray_image,CV_BGR2GRAY);

}

//void culculate_optical_flow::set_clip_images(const int& nh=cnh,const int& nw=cnw){//step2
void culculate_optical_flow::set_clip_images(void){
  for(int i=0;i<cnh;i++){
    for(int j=0;j<cnw;j++){
      clp_img[i][j]=pre_gray_image(cv::Rect((int)(j*(width)/cnw),(int)(i*(height)/cnh),(int)(width/cnw),(int)(height/cnh)));
    }
  }
}
//bool culculate_optical_flow::obtain_feature_points(const cv::Mat& cur_depth_image,const int& nh=cnh,const int& nw=cnw){
bool culculate_optical_flow::obtain_feature_points(const cv::Mat& pre_depth_image){
  //nh=nw=16 -> fp.size:987
  int clp_max_points=max_points/(cnh*cnw)*4;
  auto detector = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
  cv::Point2i ppts;
  float ppre_z;
	float y;
	const float y_th=0.1;
	//METHOD1
  for(int i=0;i<cnh-cnh/5;i++){
    for(int j=0;j<cnw;j++){
      detector.detect(clp_img[i][j], keypoints);
      for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
        itk != keypoints.end(); ++itk){
        ppts.x=j*width/cnw+itk->pt.x;
        ppts.y=i*height/cnh+itk->pt.y;
        ppre_z=pre_depth_image.at<float>(
          ppts.y,
          ppts.x
          );
//        if(!std::isnan(ppre_z)&&!std::isinf(ppre_z)&&ppre_z>=0.5){
        if(ppre_z>=0.5&&!std::isinf(ppre_z)){
					y=(height/2-i)*ppre_z/f;
					if(y+0.23>y_th&&y+0.23<=3.0){
		        pts.push_back(ppts);
		        pre_z.push_back(ppre_z);
					}
        }//end if
      }//end for
    }//end for
  }//end for
  //METHOD2
	/*
	int count;
	const int COUNT_MAX=30;
  for(int i=0;i<cnh-cnh/5;i++){
    for(int j=0;j<cnw;j++){
			count=0;
      detector.detect(clp_img[i][j], keypoints);
      for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
        itk != keypoints.end(); ++itk){
        ppts.x=j*width/cnw+itk->pt.x;
        ppts.y=i*height/cnh+itk->pt.y;
        ppre_z=pre_depth_image.at<float>(
          ppts.y,
          ppts.x
          );
        if(ppre_z>=0.5){
					y=(height/2-i)*ppre_z/f;
					if(y+0.23>y_th&&y+0.23<=3.0){
		        pts.push_back(ppts);
		        pre_z.push_back(ppre_z);
						count++;
		        if(count>=COUNT_MAX){
							break;
						}
					}
        }//end if
      }//end for
    }//end for
  }//end for
	*/
  ///METHOD 3
	/*
  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
      ppts.x=i;
      ppts.y=j;
      ppre_z=pre_depth_image.at<float>(
        ppts.y,
        ppts.x
        );

      if(ppre_z>=0.5){
				y=(height/2-i)*ppre_z/f;
				if(y+0.23>y_th&&y+0.23<=3.0){
	        pts.push_back(ppts);
	        pre_z.push_back(ppre_z);
				}
      }//end if
    }//end for
  }//end for
	*/

  if(!pts.size()){
    return false;
  }
	std::cout<<"pts.size():"<<pts.size()<<"\n";
  return true;
}
void culculate_optical_flow::culculating_observed_opticalflow(void){
  //return true:pts.size()>0,false:pts.size()==0
  cv::calcOpticalFlowPyrLK(pre_gray_image,cur_gray_image, pts, npts, sts, ers, cv::Size(window_size,window_size), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);

}
void culculate_optical_flow::culculating_moving_objects_opticalflow(const cv::Mat& cur_depth_image,const double& w_v,const double& dyaw,const double& dt){//+V,Ω
  float pcur_z,ppre_z;
  cv::Point2i ppt;
  ::obst_avoid::point3d vX_element;
  ::obst_avoid::img_point pt;
	float X,Y;
//	std::cout<<"w_v,dyaw,dt):("<<w_v<<","<<dyaw<<","<<dt<<")\n";

  for(int i=0;i<pts.size();i++){
    if(sts[i]&&npts[i].x>=0&&npts[i].y>=0){
      pcur_z=cur_depth_image.at<float>(
          (int)npts[i].y,
          (int)npts[i].x
          );
			/*
			if(npts[i].y<0||npts[i].x<0)
			{
				while(ros::ok()){
					std::cout<<"npts["<<i<<"]:"<<npts[i]<<"\n";
					std::cout<<"pcur_z:"<<pcur_z<<"\n";
					std::cout<<"(int)npts[i]:"<<(int)npts[i].y<<","<<(int)npts[i].x<<"\n";
					std::cout<<"sts[i]:"<<sts[i]<<"\n";
					
				}
				
			}
			*/
      if(!std::isinf(pcur_z)&&pcur_z>=0.5){
        X=(float)(pts[i].x-width/2.0);//-width;
    	//	Y=(float)(pts[i].y-height/2.0);//-height;
       	Y=(float)(height/2.0-pts[i].y);//-height;
         //画像ヤコビアンではなく並進と回転行列で計算
      ppt.x=pts[i].x- (float)(//wheel only
        w_v*sin(-dyaw)*dt/pre_z[i]-X/pre_z[i]*w_v*cos(-dyaw)*dt
        -(f+pow(X,2.0)/f)*dyaw
        );

      ppt.y=pts[i].y-(float)(
          -(Y/pre_z[i]*w_v*cos(-dyaw)*dt)
          -(X*Y*dyaw/f
          ));
//	std::cout<<"pts[i],ppt,npts[i]:"<<pts[i]<<","<<ppt<<","<<npts[i]<<"\n";
      points.push_back(pts[i]);
			cv::Point2i newpoints_temp;
			//newpoints_temp.x=(int)(npts[i].x-ppt.x)+pts[i].x;
			//newpoints_temp.y=(int)(npts[i].y-ppt.y)+pts[i].y;
			newpoints_temp.x=(int)npts[i].x;
			newpoints_temp.y=(int)npts[i].y;
      newpoints.push_back(newpoints_temp);
			z.push_back(pre_z[i]);
			nz.push_back(cur_z[i]);
			
			
      //vX_element.x=(npts[i].x*pcur_z/f - ppt.x*pre_z[i]/f)/dt;
      //vX_element.y=(npts[i].y*pcur_z/f - ppt.y*pre_z[i]/f)/dt;
      vX_element.x=(npts[i].x*pcur_z/f - pts[i].x*pre_z[i]/f - w_v*sin(-dyaw))/dt;
      vX_element.y=(npts[i].y*pcur_z/f - pts[i].y*pre_z[i]/f)/dt;
      vX_element.z=(pcur_z-pre_z[i])/dt-w_v*cos(-dyaw);
      vX.vel.push_back(vX_element);
      
      pt.h=(int)npts[i].y;
      pt.w=(int)npts[i].x;
			/*
			if(pt.h<0||pt.w<0)
			{
				while(ros::ok()){
					std::cout<<"npts["<<i<<"]:"<<npts[i]<<"\n";
				}
				
			}
			*/
      vX.pt.push_back(pt);
      //				newpoints.push_back(npts[i]);

      }
    }
  }
}

void culculate_optical_flow::publish_objects_velocity(void){
	pub_vel.publish(vX);
}

void culculate_optical_flow::publish_flow_image(const cv::Mat& cur_image){
  view_image=cur_image.clone();
  for(int i=0;i<points.size();i++){
    cvArrow(&view_image,
    cv::Point( (int)(points[i].x),
      (int)(points[i].y) ),
    cv::Point( (int)(newpoints[i].x),
      (int)(newpoints[i].y) ),
    cv::Scalar(0,200,200) );//
  }

  //publish_debug_image(view_image);
}
void culculate_optical_flow::clear_vector(void){
  pts.clear();   //特徴点
  npts.clear();  //移動後の特徴点
  sts.clear();
  ers.clear();
  keypoints.clear();
  pre_z.clear();
  cur_z.clear();
  points.clear();
  newpoints.clear();
  z.clear();
  nz.clear();

  jnpts.clear();
  jnewpoints.clear();
  tracking_count_p.clear();
  tracking_count.clear();
  vX.vel.clear();
  vX.pt.clear();
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

  image_class img_cls;
	depth_image_class depth_img_cls;
  visual_odometry_class odm_cls;
  wheel_odometry_class wodm_cls;
  time_class time_cls;
  culculate_optical_flow cul_optflw;

	img_cls.define_variable();
	depth_img_cls.define_variable();
	odm_cls.define_variable();
	wodm_cls.define_variable();

  while(ros::ok()){
		std::cout<<"1\n";
    img_cls.set_image();
		std::cout<<"2\n";
    depth_img_cls.set_image();
		std::cout<<"3\n";
    time_cls.set_time();
		std::cout<<"4\n";
    odm_cls.set_odometry();
		std::cout<<"5\n";
    odm_cls.set_delta_odometry();
		std::cout<<"6\n";
    wodm_cls.set_delta_odometry(time_cls.get_delta_time());
		std::cout<<"7\n";
		if(img_cls.is_pre_image()&&depth_img_cls.is_pre_image()){
		  cul_optflw.set_gray_images(img_cls.get_pre_image_by_ref(),img_cls.get_cur_image_by_ref());
			std::cout<<"8\n";
		  cul_optflw.set_clip_images(/*default*/);
			std::cout<<"9\n";
		  if(cul_optflw.obtain_feature_points(depth_img_cls.get_pre_image_by_ref()) ){
				std::cout<<"10\n";
				cul_optflw.culculating_observed_opticalflow();
				std::cout<<"11\n";
				cul_optflw.culculating_moving_objects_opticalflow(depth_img_cls.get_pre_image_by_ref(),wodm_cls.get_wheel_velocity(),odm_cls.get_delta_yaw(),time_cls.get_delta_time());
				std::cout<<"12\n";
				cul_optflw.publish_flow_image(img_cls.get_cur_image_by_ref());
				std::cout<<"13\n";
				img_cls.publish_debug_image(cul_optflw.get_view_image());
				std::cout<<"14\n";
				std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
		    cul_optflw.publish_objects_velocity();
			}
		}
    cul_optflw.clear_vector();
  }
  return 0;
}
