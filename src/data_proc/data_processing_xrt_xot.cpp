#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<fstream>
#include"obst_avoid/dvw.h"
#include <ros/callback_queue.h>
#include<nav_msgs/Odometry.h>
#include<obst_avoid/robot_odm.h>
#include<obst_avoid/select_theta.h>

class data_prcs_exp
{

	ros::NodeHandle nh1,nh2,nh3;
//subscriber
	ros::Subscriber sub1,sub2,sub3;//DepthImage
	ros::CallbackQueue queue1,queue2,queue3;
//subscribe options
	ros::SubscribeOptions dvw_option;
public:	
	nav_msgs::Odometry oodom;
	obst_avoid::robot_odm rodom;
	obst_avoid::select_theta stheta;
	double dt;
	data_prcs_exp(){
		oodom.pose.pose.position.x=0;
		oodom.pose.pose.position.y=0;
		oodom.pose.pose.position.z=0;
		rodom.x=0;		
		rodom.y=0;		
		rodom.th=0;		
		stheta.select_vel=0;
		stheta.select_theta=0;
		nh1.setCallbackQueue(&queue1);
		nh2.setCallbackQueue(&queue2);
		nh3.setCallbackQueue(&queue3);
		sub1=nh1.subscribe("/odom",1,&data_prcs_exp::oodom_callback,this);
		sub2=nh2.subscribe("/robot_odm",1,&data_prcs_exp::robot_odm_callback,this);
		sub3=nh3.subscribe("/select_theta",1,&data_prcs_exp::select_theta_callback,this);
	}
	~data_prcs_exp(){
		
	}
	void oodom_callback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		oodom.pose=msg->pose;
	}
	void robot_odm_callback(const obst_avoid::robot_odm::ConstPtr& msg)
	{
		rodom.x=msg->x;		
		rodom.y=msg->y;		
		rodom.th=msg->th;	
	}
	void select_theta_callback(const obst_avoid::select_theta::ConstPtr& msg)
	{
		stheta.select_vel=msg->select_vel;
		stheta.select_theta=msg->select_theta;
	}
	void call(void){
			queue1.callOne(ros::WallDuration(0.02));
			queue2.callOne(ros::WallDuration(0.02));
			queue3.callOne(ros::WallDuration(0.02));
	}
	void fprint_data(void){
		std::ofstream ofsss("data_ex.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofsss<<oodom.pose.pose.position.x<<","
			<<oodom.pose.pose.position.y<<","
			<<oodom.pose.pose.position.z<<","
			<<","
			<<rodom.x<<","
			<<rodom.y<<","
			<<rodom.th<<","
			<<","
			<<stheta.select_vel<<","
			<<stheta.select_theta<<","
			<<std::endl;
	}
};

	int main(int argc,char **argv){
		ros::init(argc,argv,"data_prcessing_xrt_and_xot");
		
		data_prcs_exp prc;
		std::ofstream ofsss("data_ex.csv",std::ios::app);//("./Documents/data_ex.csv",std::ios::app);
		ofsss<<"oodom_x"<<","
			<<"oodom_y"<<","
			<<"oodom_z"<<","
			<<","
			<<"rodom.x"<<","
			<<"rodom.y"<<","
			<<"rodom.th"<<","
			<<","
			<<"select_vel"<<","
			<<"select_theta"<<","
			<<std::endl;

		while(ros::ok()){
			prc.call();
			prc.fprint_data();
		}
		return 0;
}
