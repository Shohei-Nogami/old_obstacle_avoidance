#include"img_prc_cls.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"img_prc");
	//コンストラクタにて初期化
	ImageProcesser prc;
	prc.reserve_vectors();
	//while文でloop
	ros::Rate rate(10);
	while(ros::ok()){
		prc.set_image();	//get original image
		prc.set_depth();	//get depth image
		prc.set_odom();		//get odometry, dx, dz and dt
		prc.set_wodom();	//get wheel odometry
		prc.filtering_depthimage();
		prc.pub_depthimg();	//publish depth image
//		prc.show_speed();	//print dx, dz, dt, vx and vz to terminal 
		prc.imageProcess();	//tracking and adding feature points , culculating opticalflow
		
		prc.renew_vectors();	//renew pts, pz, tracking_count_p
		prc.clear_vectors();	//clear npts, sts, ers, keypoints, cp[i][j], jnpts

//		prc.vector_field_histgram();

//		ROS_INFO("prc.setave3d();");
		prc.set_ave3d();
//		ROS_INFO("prc.setave3d();,prc.pub_response();");
		prc.pub_response();
//		ROS_INFO("prc.pub_response();");
		prc.prd_process();
		prc.wheel_control();
		prc.clear_dtctvectors();
		prc.pub_left_img();
//		rate.sleep();
//		prc.print_points_size();
//		prc.print_dt();
//		prc.print_sp3dsize();
///		prc.print_imgdt();
		prc.write_odom();
//		prc.show_speed();
//		prc.print_bias();
//		prc.print_w();
//		prc.print_clpsize();
		prc.pub_target_point();

	}
	return 0;
}

