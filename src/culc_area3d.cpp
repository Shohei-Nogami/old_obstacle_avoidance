#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<fstream>
#include <std_msgs/Empty.h>
#include"obst_avoid/point3d"
#include"obst_avoid/line_point3d"
#include"obst_avoid/sqr_point3d"
//subscriber
	ros::Subscriber sub_depth;//DepthImage
	ros::Subscriber sub_empty;
	ros::CallbackQueue depth_queue;
	ros::CallbackQueue empty_queue;
//subscribe options
	ros::SubscribeOptions depth_option;
	ros::SubscribeOptions empty_option;
	
	
	cv::bridge depthimg;
	cv::btidge pubdepthimg;
	cv::Mat depth_img;
	obst_avoid::point3d p3d
	obst_avoid::line_point3d lp3d
	obst_avoid::sqr_point3d sp3d
	const int cn=12;
	const int cnw=cn*2;
	const int cnh=cn;
	const double f=350;
	bool rf=false;
	const int width=672;
	const int height=376;
	void depth_callback(const sensor_msgs::ImageConstPtr& msg)
	{
	  lp3d.line_p3d.clear();
	  sp3d.sqr_p3d.clear();
	    try{
		depthimg= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depth_img=depthimg->image;
	    }
	    catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
		msg->encoding.c_str());
		return ;
	    }
	}
	void culc_ave(void){
//	  int cn=12;
//	  cnw=cn*2;
//	  cnh=cn;
	  double ave_depth[cnh][cnw];
	  int nan_count,count;
	  long double sum_depth;
	  for(int i=0;i<cnh;i++){
	    for(int j=0;j<cnw;j++){
	      	count=0;
	      nan_count=0;
	      sum_depth=0;
	      for(int h=(int)(i*height/cnh);h<(int)((i+1)*height/cnh));h++){
	        for(int w=(int)(j*width/cnw);w<(int)((j+1)*width/cnw));w++){
	          double depth=depth_img.at<float>(h,w);
	          if(!std::isnan(depth))
	            sum_depth+=depth;
	          else
	            nan_count++;
	          count++;
	        }
	      }
	      ave_depth[i][j]=sum_depth/(count-nan_count);
	      p3d.x=( (double)j*width/cnw+(double)width/cnw/2 )*ave_depth[i][j]/f;
	      p3d.y=( (double)j*height/cnh+(double)height/cnh/2 )*ave_depth[i][j]/f;
	      p3d.z=ave_depth[i][j];
	      lp3d.push_back(p3d);
	    }
	    sp3d.push_back(lp3d);
	  }
	}
	void empty_callback(const std_msgs::Empty& msg){
	  rf=true;
	}
	
	int main(void){
	  ros::NodeHandle nh,nh1,nh2;
//	  image_transport::ImageTransport it(nh);
	  pub_dpt=nh.advertise<obst_avoid::sqr_point3d>("ave_p3d",1);

		nh1.setCallbackQueue(&depth_queue);
	nh2.setCallbackQueue(&empty_queue);	sub_depth=nh1.subscribe("output_dptimage",1,depth_callback);
		sub_empty=nh2.subscribe("empty_msg",1,empty_callback);
		lp3d.line_p3d.reserve(cnw);
		sp3d.sqr_p3d.reserve(cnh);
		
	  while(ros::ok()){
	    depth_queue.callOne(ros::WallDuration(1));
	    if(depth_img.empty())
	      continue;
	    //
	    culc_ave();
	    rf=false;
	    while(ros::ok()&&rf==false){
	      pub_dpt.publish(sp3d);
	      empty_queue.callOne(ros::WallDuration(0.005));
	    }
	    //publish 
	    /*cv_bridge::CvImagePtr pubdepthimg(new cv_bridge::CvImage);
		pubdepthimg->encoding=sensor_msgs::image_encodings::MONO8;
		pubdepthimg->image=depth_img.clone();
	    pub_dpt.publish(pubdepthimg->toImageMsg());*/
	  }
	  return 0;
	}

