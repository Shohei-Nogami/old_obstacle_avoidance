#include"img_prc_cls.h"

ros::Time img_crnt_time,odm_crnt_time,img_prvs_time,odm_prvs_time;
long double img_dt,odm_dt;
bool prcs_onc;

int main(int argc,char **argv){
	ros::init(argc,argv,"img_prc");
	//コンストラクタにて初期化
	ImageProcesser prc;
	prcs_onc=false;
	//while文でloop
	while(ros::ok()){
		//imageを取得
//		ROS_INFO("image service call");
		if(prc.callimgsrv()){
			//ROS_INFO("respons by image service ");
			ROS_INFO("1 ");
		}
		if(prc.set_lpfimg()){
			ROS_INFO("2 ");
			//depthを取得
			//ROS_INFO("depth service call");
			if(prc.calldptsrv()){
			ROS_INFO("3 ");
				prc.setdepth();
//				prc.pub_depthimg();
			}
			//odometryを取得
			//ROS_INFO("odometry service call");
			if(prc.callodmsrv()){
			ROS_INFO("4 ");
				prc.setodom();
			}
			ROS_INFO("5 ");
			//各パラメータset
/*			prc.setimage();
			ROS_INFO("6 ");
			prc.setdepth_img();
			ROS_INFO("7 ");
			//publish original image
			prc.pub_org_img();
			ROS_INFO("8 ");
			//画像処理
			prc.imageProcess();
*/
			prc.reset_img_srv_count();
		}
	}
	return 0;
}
