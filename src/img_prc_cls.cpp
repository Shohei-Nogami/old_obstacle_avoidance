#include"img_prc_cls.h"

ImageProcesser::ImageProcesser()//
	:it(nh)
	{
		start_time = ros::Time::now();
		pub_orgimg=it.advertise("output_originalimage",1);
		pub_Limg=it.advertise("output_Limage",1);
		pub_Lmsk=it.advertise("output_Mskimage",1);
		pub_dpt=it.advertise("output_dptimage",1);

		nh1.setCallbackQueue(&image_queue);
		nh2.setCallbackQueue(&depth_queue);
		nh.setCallbackQueue(&odom_queue);
		nh3.setCallbackQueue(&wodom_queue);

		sub_Limg=nh1.subscribe("/zed/left/image_rect_color",1,&ImageProcesser::image_callback,this);
		sub_depth=nh2.subscribe("/zed/depth/depth_registered",1,&ImageProcesser::depth_callback,this);
		sub_odom=nh.subscribe("/zed/odom",1,&ImageProcesser::odom_callback,this);
		sub_wodom=nh3.subscribe("/wheel_data",1,&ImageProcesser::wheelodom_callback,this);
	}

