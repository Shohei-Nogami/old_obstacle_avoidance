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
cv_bridge::CvImagePtr dpt_bridge;
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//    ROS_INFO("depth received by subscriber");
    try{
        dpt_bridge= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
        msg->encoding.c_str());
        return ;
    }
}

bool dpt_srv(obst_avoid::image::Request& req,obst_avoid::image::Response& res)
{
//	ROS_INFO("called depth service");
	dpt_bridge->toImageMsg(res.imgmsg);
	return true;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"dpt_srv");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ros::ServiceServer service 
		= nh.advertiseService("getdpt",dpt_srv);
	image_transport::Subscriber sub_depth=it.subscribe("/zed/depth/depth_registered",1,
		&depthImageCallback);
	ros::spin();
	return 0;
}
