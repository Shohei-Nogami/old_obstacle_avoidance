#include"img_prc_cls.h"

//debug
	void ImageProcesser::print_odom(void){
		ROS_INFO("(x,y,z,|,r,p,y):(%f,%f,%f,|,%f,%f,%f)",
			position_x,position_y,position_z,roll,pitch,yaw);
	}
	//show current speed
	void ImageProcesser::show_speed(void){
		ROS_INFO("(v,w:(%f,%f),(dz,dw,dt):(%f,%f,%f)",global_dx/dt,dyaw/dt,global_dx,dyaw,dt);
	}
	void ImageProcesser::print_dt(void){
		ROS_INFO("(dt,fps):(%f,%f)",dt,1/dt);
	}
	void ImageProcesser::print_imgdt(void){
		ROS_INFO("(imgdt,imgfps,img_dt-dt):(%f,%f,%f)",img_dt,1/img_dt,img_dt-dt);
	}
	void ImageProcesser::print_points_size(void){
		std::cout<<"points size:(prev,cur)-"<<points.size()<<","<<newpoints.size()<<"\n";
	}
	

