#include"img_prc_cls.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"test");
	//コンストラクタにて初期化
	ImageProcesser prc;

	//while文でloop
	while(ros::ok()){
		//imageを取得
		if(prc.callimgsrv()){
			prc.setimage();
			prc.pub_org_img();
		}
		//depthを取得
		if(prc.calldptsrv()){
			prc.setdepth();
			prc.pub_depthimg();
		}
	}

	return 0;
}
