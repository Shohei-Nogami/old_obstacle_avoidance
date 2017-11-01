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
	void ImageProcesser::print_bias(void){
		std::cout<<"bias:(visual,order,delta)-"<<dyaw*f<<","<<w_dyaw*f<<","<<std::abs(dyaw-w_dyaw)*f<<"\n";
	}
	void ImageProcesser::print_w(void){
		std::cout<<"(w_w,v_w):"<<w_dyaw/dt<<","<<dyaw/dt<<"\n";
	}
	void ImageProcesser::print_clpsize(void){
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				std::cout<<"cp_size["<<i<<","<<j<<"]:"<<(int)cp[i][j].size()<<"\n";
				int size=(int)cp[i][j].size();
				if(size >= clp_point_size)
					std::cout<<"size over\n";
			}
		}
	}


