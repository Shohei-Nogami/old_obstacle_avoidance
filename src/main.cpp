#include"img_prc_cls.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"img_prc");
	//コンストラクタにて初期化
	ImageProcesser prc;
	prc.reserve_vectors();
	//while文でloop
	ros::Rate rate(10);
	while(ros::ok()){
		prc.setimage();
		prc.set_depth();
		prc.setodom();
//		prc.setwodom();
//		ROS_INFO("prc.pub_depthimg();");
		prc.pub_depthimg();
//		ROS_INFO("prc.pub_depthimg();");
//		prc.approx_depth_img();
//		prc.show_speed();
		prc.imageProcess();
//		prc.print_points_size();
//		prc.pub_org_img();
//		prc.pub_left_img();
//		prc.print_points_size();
//		prc.wheel_control();
		prc.renew_vectors();
//		prc.print_points_size();
		prc.clear_vectors();
//		ROS_INFO("prc.setave3d();");
		prc.setave3d();
//		ROS_INFO("prc.setave3d();,prc.pub_response();");
		prc.pub_response();
//		ROS_INFO("prc.pub_response();");
		prc.prd_prcess();
		prc.clear_dtctvectors();
		prc.pub_left_img();
//		rate.sleep();
//		prc.print_points_size();
//		prc.print_dt();
//		prc.print_sp3dsize();
		prc.print_imgdt();
//		prc.write_odom();
		prc.print_odom();
//		prc.show_speed();
//		prc.print_bias();
//		prc.print_w();
//		prc.print_clpsize();
	}
	return 0;
}

