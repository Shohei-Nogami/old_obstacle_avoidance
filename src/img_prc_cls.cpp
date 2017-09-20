#include"img_prc_cls.h"

ImageProcesser::ImageProcesser()//
	:it(nh),PROCESS_ORDER(1),ODOMETRY_RECEIVED(false),width(672),height(376),max_value_x(0),min_value_x(100),max_value_y(0),min_value_y(100)//width(2560),height(720)//,width(3840),height(1080)
{
	start_time = ros::Time::now();
	pub_orgimg=it.advertise("output_originalimage",1);
	pub_Limg=it.advertise("output_Limage",1);
	pub_Lmsk=it.advertise("output_Mskimage",1);
	pub_dpt=it.advertise("output_dptimage",1);
	imgclient = nh.serviceClient<obst_avoid::image>("getimg");
	dptclient = nh.serviceClient<obst_avoid::image>("getdpt");
	odmclient = nh.serviceClient<obst_avoid::odometry>("getodm");

}
