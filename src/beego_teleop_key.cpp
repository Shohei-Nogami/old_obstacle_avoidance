#include"ros/ros.h"
#include"obst_avoid/wheel_msg.h"
#include <geometry_msgs/Twist.h>
	
	ros::Subscriber sub_twist;
	ros::Publisher pub_wheel;
	int vel=200;
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
	ros::NodeHandle n;
	pub_wheel=n.advertise<obst_avoid::wheel_msg>("wheel_data",1000);
	int flag=1;
	obst_avoid::wheel_msg wheelMsg;
	std::cout<<"linear.x,angular.z:("<<msg->linear.x<<","<<msg->angular.z<<")\n";
/*		if(msg->linear.x==0&&msg->angular.z==0)
			wheelMsg.vel_data="00000000";
		else if(msg->linear.x>0&&msg->angular.z==0)
			wheelMsg.vel_data="01000100";
		else if(msg->linear.x==0&&msg->angular.z<0)
			wheelMsg.vel_data="-1000100";//left=100,right=000
		else if(msg->linear.x==0&&msg->angular.z>0)
			wheelMsg.vel_data="0100-100";//left=000,right=100
		else
			wheelMsg.vel_data="00000000";
*/
	if(vel==100){
		if(msg->linear.x==0&&msg->angular.z==0)
			wheelMsg.vel_data="00000000";
		else if(msg->linear.x>0&&msg->angular.z==0)
			wheelMsg.vel_data="01000100";
		else if(msg->linear.x==0&&msg->angular.z<0)
			wheelMsg.vel_data="00000100";//left=100,right=000 curve
		else if(msg->linear.x==0&&msg->angular.z>0)
			wheelMsg.vel_data="01000000";//left=000,right=100 curve
		else
			wheelMsg.vel_data="00000000";
	}
	if(vel==200){
		if(msg->linear.x==0&&msg->angular.z==0)
			wheelMsg.vel_data="00000000";
		else if(msg->linear.x>0&&msg->angular.z==0)
			wheelMsg.vel_data="02000200";
		else if(msg->linear.x==0&&msg->angular.z<0)
			wheelMsg.vel_data="00000200";//left=100,right=000 curve
		else if(msg->linear.x==0&&msg->angular.z>0)
			wheelMsg.vel_data="02000000";//left=000,right=100 curve
		else
			wheelMsg.vel_data="00000000";
	}
	if(vel==150){
		if(msg->linear.x==0&&msg->angular.z==0)
			wheelMsg.vel_data="00000000";
		else if(msg->linear.x>0&&msg->angular.z==0)
			wheelMsg.vel_data="02000200";
		else if(msg->linear.x==0&&msg->angular.z<0)
			wheelMsg.vel_data="01000200";//left=100,right=000 curve
		else if(msg->linear.x==0&&msg->angular.z>0)
			wheelMsg.vel_data="02000100";//left=000,right=100 curve
		else
			wheelMsg.vel_data="00000000";
	}


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
