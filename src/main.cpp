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
			prc.setimage();

			img_crnt_time=ros::Time::now();
    		if(prcs_onc){
        		ros::Duration imgdt=img_crnt_time-img_prvs_time;
        		img_dt= imgdt.toSec();
			}
        	img_prvs_time=img_crnt_time;

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
		    odm_crnt_time=ros::Time::now();
		    if(prcs_onc){
		        ros::Duration odmdt=odm_crnt_time-odm_prvs_time;
		        odm_dt= odmdt.toSec();
		    }
		    else
		        prcs_onc=true;
			odm_prvs_time=odm_crnt_time;
		}
//		std::cout<<"dt(img,odm):("<<img_dt<<","<<odm_dt<<")"<<std::endl;	
		//show current speed
		prc.show_speed();
		//画像処理
		prc.imageProcess();
	}

	return 0;
}
