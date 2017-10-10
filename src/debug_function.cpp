#include"img_prc_cls.h"

//debug
	void ImageProcesser::print_odom(void){
		ROS_INFO("(x,y,z,|,r,p,y):(%f,%f,%f,|,%f,%f,%f)",
			position_x,position_y,position_z,roll,pitch,yaw);
	}
	//show current speed
	void ImageProcesser::show_speed(void){
		ROS_INFO("(v,w:(%f,%f),(dz,dw,dt):(%f,%f,%f)",global_dx/dt,dyaw/dt,global_dz,dyaw,dt);
	}
