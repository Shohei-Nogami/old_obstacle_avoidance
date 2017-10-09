#include"img_prc_cls.h"

ImageProcesser::ImageProcesser()//
	:it(nh),it1(nh1),it2(nh2),width(672),height(376),max_value_x(0),min_value_x(100),max_value_y(0),min_value_y(100)//width(2560),height(720)//,width(3840),height(1080)
	{
		start_time = ros::Time::now();
		pub_orgimg=it.advertise("output_originalimage",1);
		pub_Limg=it.advertise("output_Limage",1);
		pub_Lmsk=it.advertise("output_Mskimage",1);
		pub_dpt=it.advertise("output_dptimage",1);
		
		nh1.setCallbackQueue(&image_queue);
		nh2.setCallbackQueue(&depth_queue);
		nh.setCallbackQueue(&odom_queue);
		
		
		
//		image_option= ros::SubscribeOptions::create<sensor_msgs::Image>("/zed/left/image_rect_color",1,image_callback,ros::VoidPtr(),&image_queue);
//		depth_option= ros::SubscribeOptions::create<sensor_msgs::Image>("/zed/depth/depth_registered",1,depth_callback,ros::VoidPtr(),&depth_queue);
//		odom_option= ros::SubscribeOptions::create<nav_msgs::Odometry>("/zed/odom",1,odom_callback,ros::VoidPtr(),&odom_queue);
//		nh1.setCallbackQueue(&image_queue);
//		nh2.setCallbackQueue(&depth_queue);
		
//		sub_Limg=nh.subscribe(image_option);
//		sub_depth=nh.subscribe(depth_option);
//		sub_odom=nh.subscribe(odom_option);
//		sub_Limg=it1.subscribe("/zed/depth/depth_registered",1,depth_callback);
//		sub_depth=it2.subscribe("/zed/depth/depth_registered",1,depth_callback);
//		sub_odom=nh.subscribe(odom_option);
		sub_Limg=nh1.subscribe("/zed/left/image_rect_color",1,&ImageProcesser::image_callback,this);
		sub_depth=nh2.subscribe("/zed/depth/depth_registered",1,&ImageProcesser::depth_callback,this);
		sub_odom=nh.subscribe("/zed/odom",1,&ImageProcesser::odom_callback,this);
	}
