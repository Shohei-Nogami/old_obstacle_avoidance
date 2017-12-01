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
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				std::cout<<"cp_size["<<i<<","<<j<<"]:"<<(int)cp[i][j].size()<<"\n";
				int size=(int)cp[i][j].size();
				if(size >= clp_point_size)
					std::cout<<"size over\n";
			}
		}
	}
	void ImageProcesser::print_cptsize(void){
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
			  std::cout<<"cpt["<<i<<"]["<<j<<"].size:"<<cpt[i][j].size()<<"\n";

			}
		}
	}
	void ImageProcesser::print_sp3dsize(void){
		std::cout<<"i=cnh=sp3d.sqr_p3d.size():"<<sp3d.sqr_p3d.size()<<"\n";
		std::cout<<"j=cnw=sp3d.sqr_p3d[0].line_p3d.size():"<<sp3d.sqr_p3d[0].line_p3d.size()<<"\n";

	}
	void ImageProcesser::write_odom(void){
	  std::ofstream ofsss("./Documents/odometry.csv",std::ios::app);
			ofsss<<position_x<<","
			  <<position_y<<","
			  <<position_z<<","
			  <<new_time<<","
			  <<std::endl;
	}
	bool ImageProcesser::culc_ave_dyaw(void){
		vec_dyaw.push_back(dyaw);
//		std::cout<<"vec_dyaw.size():"<<vec_dyaw.size()<<"\n";
//		if((int)vec_dyaw.size()>5000){
			ave_dyaw=0;
			for(int i=0;i<vec_dyaw.size();i++){
				ave_dyaw+=vec_dyaw[i];
				
			}
			ave_dyaw=ave_dyaw/(int)vec_dyaw.size();
			ROS_INFO("ave_dyaw:%f",ave_dyaw);
			return true;
//		}
		return false;		
	}

