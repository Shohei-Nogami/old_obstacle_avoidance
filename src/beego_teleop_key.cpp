#include"ros/ros.h"
#include"obst_avoid/wheel_msg.h"
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>

	ros::Subscriber sub_twist;
	ros::Publisher pub_wheel;
	int maxvel=600;
	std::string order;
	int vel_r=0;
	int vel_l=0;
	int target_vel_r=0;
	int target_vel_l=0;
	obst_avoid::wheel_msg wheelMsg;
	int pbtn=0;
	int cbtn;
	const int ac=50;
	int pvel_r=0;
	int pvel_l=0;
	int T;
	ros::CallbackQueue twist_queue;
	ros::Time start_time;
	double prev_time;	
	double new_time;	//
	double dt;	
	
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
		  
//		  wheelMsg.vel_data="00000000";
//			wheelMsg.vel_l=0;
//			wheelMsg.vel_r=0;
			target_vel_l=0;
			target_vel_r=0;
		}
		else if(msg->linear.x>0&&msg->angular.z==0){


		  target_vel_r+=ac;
		  target_vel_l+=ac;		  
		  if(target_vel_r>=maxvel)
			target_vel_r=maxvel;
		  if(target_vel_l>=maxvel)
			target_vel_l=maxvel;
//		  generation_order();
		}
		else if(msg->linear.x==0&&msg->angular.z<0){

		  target_vel_r-=ac/2;
		  target_vel_l+=ac/2;
		  if(target_vel_r>=maxvel)
			target_vel_r=maxvel;
		  if(target_vel_l<=-maxvel)
			target_vel_l=-maxvel;
//		  generation_order();
		}
		else if(msg->linear.x==0&&msg->angular.z>0){

		  target_vel_r+=ac/2;
		  target_vel_l-=ac/2;
		  if(target_vel_l>=maxvel)
			target_vel_l=maxvel;
		  if(target_vel_r<=-maxvel)
			target_vel_r=-maxvel;
//		  generation_order();
		}
		else{

		  target_vel_r-=ac;
		  target_vel_l-=ac;		  
		  if(target_vel_r<=-maxvel)
			target_vel_r=-maxvel;
		  if(target_vel_l<=-maxvel)
			target_vel_l=-maxvel;
//		  generation_order();
		}
		vel_r=target_vel_r;
		vel_l=target_vel_l;
		generation_order();
		std::cout<<"msg:"<<wheelMsg.vel_data<<'\n';
		pub_wheel=n.advertise<obst_avoid::wheel_msg>("wheel_data",1);
		pub_wheel.publish(wheelMsg);
		
//		getprevtime();
}
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

int main(int argc,char **argv){
	ros::init(argc,argv,"beego_teleop_key");
	ros::NodeHandle nh;
	nh.setCallbackQueue(&twist_queue);
	sub_twist=nh.subscribe("turtle1/cmd_vel",1,twistCallback);

	start_time=ros::Time::now();
	gettime();

//	pub_wheel=nh.advertise<obst_avoid::wheel_msg>("wheel_data",1);
	std::cout<<"ready to publish\n";

	while(ros::ok()){
		twist_queue.callOne(ros::WallDuration(0.5));
/*		getprevtime();
		gettime();
		culcdt();
		double T=dt/2;
		std::cout<<"dt:"<<dt<<"\n";
//		vel_r=(T*vel_r+dt*target_vel_r)/(T+dt);
//		vel_l=(T*vel_l+dt*target_vel_l)/(T+dt);
		vel_r=target_vel_r;
		vel_l=target_vel_l;
		generation_order();
		std::cout<<"msg:"<<wheelMsg.vel_data<<'\n';
		pub_wheel.publish(wheelMsg);
*/
//		ros::spin();
	}
	return 0;
}
