#include"ros/ros.h"
//画像取得用
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>//オプティカルフロー用
#include<opencv2/features2d/features2d.hpp>
#include<typeinfo>//型調べ
#include<sensor_msgs/image_encodings.h>
//service
#include<obst_avoid/image.h>
#include<obst_avoid/sncr.h>

ros::ServiceClient synchro_client;
cv_bridge::CvImagePtr img_bridge;
obst_avoid::sncr sncr_srv;
image_transport::Subscriber sub_Limg;
//wheather received image 
bool img_rcvd;
//count image service
int count;
//LPF value
const int lpf_value = 3;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//    ROS_INFO("image received by subscriber");
	try{
		img_rcvd=true;
		img_bridge= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		std::cout<<"received image\n";
	}
	catch(cv_bridge::Exception& e) {
	        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
	        msg->encoding.c_str());
	        return ;
	}
}

bool img_srv(obst_avoid::image::Request& req,obst_avoid::image::Response& res)
{
	ros::NodeHandle nhh;
	image_transport::ImageTransport it(nhh);
//	ROS_INFO("called image service");
	img_rcvd=false;
	count=req.cnt;
	sub_Limg=it.subscribe("/zed/left/image_rect_color",1,&imageCallback);
	do{
		ros::spinOnce();
	}while(!img_rcvd);

	if(!(count<lpf_value-1)){
			if(synchro_client.call(sncr_srv))
				ROS_INFO("synchro image and depth ");		
			else
				ROS_INFO("not synchro");
			count=0;
	}

	img_bridge->toImageMsg(res.imgmsg);
		
	return true;
}

int main(int argc,char** argv){
	ros::init(argc,argv,"img_srv");
	ros::NodeHandle nh;
	synchro_client=nh.serviceClient<obst_avoid::sncr>("setdpt");
	ros::ServiceServer service
		= nh.advertiseService("getimg",img_srv);	
	ros::spin();
	return 0;
}
