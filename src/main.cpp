#include"img_prc_cls.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"img_prc");
	//コンストラクタにて初期化
	ImageProcesser prc;
	//while文でloop
	while(ros::ok()){
		//imageを取得
//		ROS_INFO("image service call");
		if(prc.callimgsrv()){
		}
		if(prc.calldptsrv()){
		}
		//odometryを取得
		if(prc.callodmsrv()){
			prc.setodom();
		}
		prc.setimage();
		prc.setdepth();
		prc.setdepth_img();
		prc.imageProcess();
		prc.pub_org_img();
		prc.pub_left_img();
	}
	return 0;
}
