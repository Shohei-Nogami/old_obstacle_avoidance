#include"ros/ros.h"
#include"obst_avoid/wheel_msg.h"
#include <geometry_msgs/Twist.h>
	
	ros::Subscriber sub_twist;
	ros::Publisher pub_wheel;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
	ros::NodeHandle n;
	pub_wheel=n.advertise<obst_avoid::wheel_msg>("wheel_data",1000);
	int flag=1;
	obst_avoid::wheel_msg wheelMsg;
	std::cout<<"linear.x,angular.z:("<<msg->linear.x<<","<<msg->angular.z<<")\n";
		if(msg->linear.x==0&&msg->angular.z==0)
			wheelMsg.vel_data="000000";
		else if(msg->linear.x>0&&msg->angular.z==0)
			wheelMsg.vel_data="100100";
		else if(msg->linear.x==0&&msg->angular.z<0)
			wheelMsg.vel_data="100000";//left=100,right=000
		else if(msg->linear.x==0&&msg->angular.z>0)
			wheelMsg.vel_data="000100";//left=000,right=100
		else
			wheelMsg.vel_data="000000";

		std::cout<<"ready to publish\n";
		std::cout<<"msg:"<<wheelMsg.vel_data<<'\n';
		pub_wheel.publish(wheelMsg);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"beego_teleop_key");
	ros::NodeHandle nh;
	sub_twist=nh.subscribe("turtle1/cmd_vel",1,twistCallback);
	ros::spin();
    return 0;
}
