#include"img_prc_cls.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"img_prc");
	//コンストラクタにて初期化
	ImageProcesser prc;
	prc.reserve_vectors();
	//while文でloop
	while(ros::ok()){
		prc.setimage();
		prc.setdepth();
		prc.setodom();
		prc.setdepth_img();
		prc.imageProcess();
		prc.pub_org_img();
		prc.pub_left_img();
		prc.clear_vectors();
		prc.print_dt();
	}
	return 0;
}
