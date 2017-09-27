#include"ros/ros.h"
//odometry取得用
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//ros msgファイル
#include<nav_msgs/Odometry.h>
//service file
#include<obst_avoid/odometry.h>
//ファイル出力用
#include<fstream>//file input output
::nav_msgs::Odometry odm;

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
	ROS_INFO("odometry received by subscribe");//deebug
	odm.pose=msg->pose;
	odm.twist=msg->twist;
}
bool odm_srv(obst_avoid::odometry::Request& req,obst_avoid::odometry::Response& res)
{
	ROS_INFO("called odom service");//deebug
	res.odmmsg=odm;
	return true;
}

int main(int argc,char** argv){
	ros::init(argc,argv,"odm_srv");
	ros::NodeHandle nh;
	ros::Subscriber sub_odom
		=nh.subscribe("/zed/odom",1,&OdometryCallback);
	ros::ServiceServer service
		= nh.advertiseService("getodm",odm_srv);
	ros::spin();
	return 0;
}
