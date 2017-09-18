#include"img_prc_cls.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"img_prc");
	//コンストラクタにて初期化
	ImageProcesser prc;

	//while文でloop
	while(ros::ok()){
		//imageを取得
		if(prc.callimgsrv())
			prc.setimage();
		//depthを取得
		if(prc.calldptsrv())
			prc.setdepth();
		//odometryを取得
		if(prc.callodmsrv())
			prc.setodom();
		//画像処理
		prc.imageProcess();
	}

	return 0;
}
