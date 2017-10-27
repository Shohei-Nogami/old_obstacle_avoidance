#include"img_prc_cls.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"img_prc");
	//コンストラクタにて初期化
	ImageProcesser prc;
	prc.reserve_vectors();
	//while文でloop
	ros::Rate rate(5);
	while(ros::ok()){
		prc.setimage();
		prc.set_depth();
		prc.setodom();
		prc.setwodom();
//		prc.approx_depth_img();
//		prc.show_speed();
		prc.imageProcess();
//		prc.print_points_size();
//		prc.pub_org_img();
		prc.pub_left_img();
//		prc.print_points_size();
		prc.renew_vectors();
//		prc.print_points_size();
		prc.clear_vectors();
//		rate.sleep();
//		prc.print_points_size();
//		prc.print_dt();
//		prc.print_imgdt();
//		prc.show_speed();
//		prc.print_bias();
//		prc.print_w();
//		prc.print_clpsize();
	}
	return 0;
}
