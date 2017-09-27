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
			prc.setimage();
			prc.pub_org_img();
		}
		//depthを取得
//		ROS_INFO("depth service call");
		if(prc.calldptsrv()){
			prc.setdepth();
			prc.pub_depthimg();
		}
		//odometryを取得
//		ROS_INFO("odometry service call");
		if(prc.callodmsrv()){
			prc.setodom();
		}
		//画像処理
		prc.imageProcess();
	}

	return 0;
}
