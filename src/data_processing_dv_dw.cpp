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

class data_prcs_dvw
{

	ros::NodeHandle nh;
//subscriber
	ros::Subscriber sub_dvw;//DepthImage
	ros::CallbackQueue dvw_queue;
//subscribe options
	ros::SubscribeOptions dvw_option;
public:	
	::obst_avoid::dvw dvw_msg;
	double dt;
	data_prcs_dvw(){
		dvw_msg.v=0;
		dvw_msg.w=0;
		dvw_msg.dv=0;
		dvw_msg.dw=0;		
		nh.setCallbackQueue(&dvw_queue);
		sub_dvw=nh.subscribe("/dvw_data",1,&data_prcs_dvw::dvw_callback,this);
	}
	~data_prcs_dvw(){
		
	}
	void dvw_callback(const obst_avoid::dvw::ConstPtr& msg)
	{
		dvw_msg.v=msg->v;
		dvw_msg.w=msg->w;
		dvw_msg.dv=msg->dv;
		dvw_msg.dw=msg->dw;
		return ;
	}
	void dvwcall(void){
			dvw_queue.callOne(ros::WallDuration(1));
	}
	void fprint_data(void){
		std::ofstream ofsss("./Documents/data_dvw.csv",std::ios::app);
		ofsss<<dvw_msg.v<<","
			<<dvw_msg.w<<","
			<<dvw_msg.dv<<","
			<<dvw_msg.dw<<","
			<<std::endl;
	}
};

	int main(int argc,char **argv){
		ros::init(argc,argv,"data_prcessing_v_and_w");
		
		data_prcs_dvw prc;
		std::ofstream ofsss("./Documents/data_dvw.csv",std::ios::app);
		ofsss<<"v"<<","
			<<"w"<<","
			<<"dv"<<","
			<<"dw"<<","
			<<std::endl;

		while(ros::ok()){
			prc.dvwcall();
			prc.fprint_data();
		}
		return 0;
}
