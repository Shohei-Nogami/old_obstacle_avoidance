#include"ros/ros.h"
#include"obst_avoid/wheel_msg.h"
#include <geometry_msgs/Twist.h>

	ros::Subscriber sub_twist;
	ros::Publisher pub_wheel;
	int maxvel=600;
	std::string order;
	int vel_r=0;
	int vel_l=0;
	obst_avoid::wheel_msg wheelMsg;
	int pbtn=0;
	int cbtn;
/*	double new_time,prev_time,dt;
	ros::Time start_time;
		void gettime(void){
		ros::Duration time = ros::Time::now()-start_time;
		new_time=time.toSec();
	}
	void getprevtime(void){
		prev_time=new_time;
	}
	void culcdt(void){
		dt=new_time-prev_time;
	}
	*/
	
void generation_order(void){
  		  std::string order_l,order_r;
		  if(vel_l<0){
			if(vel_l>-100)
				order_l="0"+std::to_string(vel_l);
			else
				order_l=std::to_string(vel_l);
		}
		  else if(vel_l>0){
			if(vel_l<100)
				order_l="00"+std::to_string(vel_l);
			else
				order_l="0"+std::to_string(vel_l);
		}
		  else
			order_l="0000";
		  
		  if(vel_r<0){
			if(vel_r>-100)
				order_r="0"+std::to_string(vel_r);				
			else
				order_r=std::to_string(vel_r);
		}
		  else if(vel_r>0){
			if(vel_r<100)
				order_r="00"+std::to_string(vel_r);
			else
				order_r="0"+std::to_string(vel_r);
		}
		  else
			order_r="0000";
		  order=order_l+order_r;
		  wheelMsg.vel_data=order;
			wheelMsg.vel_l=vel_l;
			wheelMsg.vel_r=vel_r;
  
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
//  	gettime();
//	culcdt();
//	if(dt<0.3)
//	  break;
  
	ros::NodeHandle n;

	
	std::cout<<"linear.x,angular.z:("<<msg->linear.x<<","<<msg->angular.z<<")\n";

		if(msg->linear.x==0&&msg->angular.z==0){
		  
		  wheelMsg.vel_data="00000000";
			wheelMsg.vel_l=0;
			wheelMsg.vel_r=0;
		}
		else if(msg->linear.x>0&&msg->angular.z==0){


		  vel_r+=100;
		  vel_l+=100;		  
		  if(vel_r>=maxvel)
			vel_r=maxvel;
		  if(vel_l>=maxvel)
			vel_l=maxvel;
		  generation_order();
		}
		else if(msg->linear.x==0&&msg->angular.z<0){

		  vel_r+=50;
		  vel_l-=50;
		  if(vel_r>=maxvel)
			vel_r=maxvel;
		  if(vel_l<=-maxvel)
			vel_l=-maxvel;
		  generation_order();
		}
		else if(msg->linear.x==0&&msg->angular.z>0){

		  vel_r-=50;
		  vel_l+=50;
		  if(vel_l>=maxvel)
			vel_l=maxvel;
		  if(vel_r<=-maxvel)
			vel_r=-maxvel;
		  generation_order();
		}
		else{

		  vel_r-=100;
		  vel_l-=100;		  
		  if(vel_r<=-maxvel)
			vel_r=-maxvel;
		  if(vel_l<=-maxvel)
			vel_l=-maxvel;
		  generation_order();
		}
		std::cout<<"ready to publish\n";
		std::cout<<"msg:"<<wheelMsg.vel_data<<'\n';
			pub_wheel=n.advertise<obst_avoid::wheel_msg>("wheel_data",10);
		pub_wheel.publish(wheelMsg);
//		getprevtime();
}

int main(int argc,char **argv){
	ros::init(argc,argv,"beego_teleop_key");
	ros::NodeHandle nh;
//	start_time=ros::Time::now();
//	gettime();
//	getprevtime();
	sub_twist=nh.subscribe("turtle1/cmd_vel",1,twistCallback);
	ros::spin();
	return 0;
}
