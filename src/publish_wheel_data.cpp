#include"ros/ros.h"
#include"obst_avoid/wheel_msg.h"
	
	
void publish_wheel_vel(){
	ros::NodeHandle nh;
	ros::Publisher pub_wheel;
	pub_wheel=nh.advertise<obst_avoid::wheel_msg>("wheel_data",1000);
	int flag=1;
	obst_avoid::wheel_msg msg;
	while(ros::ok()&&flag){
		do{
			std::cout<<"速度を入力してください(6文字):";std::cin>>msg.vel_data;
		}while(msg.vel_data.length()!=6);
		
		pub_wheel.publish(msg);
			
		std::cout<<"続ける:1,終了:0->";std::cin>>flag;	
	}
}

int main(int argc,char **argv){
    ros::init(argc,argv,"publish_wheel_vel");
    publish_wheel_vel();
    return 0;
}
