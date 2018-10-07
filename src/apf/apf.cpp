#include<apf.h>

APF::APF(float width,float height,float resolution)
	:max_pot(100),min_pot(-100){//,it(nh_pub){
	//マップデータの設定と初期化
	map_wf=width;
	map_hf=height;
	reso=resolution;
	map_wi=(int)(map_wf/resolution);
	map_hi=(int)(map_hf/resolution);
	cr=std::sqrt(2)*reso/2;
	
	
	//cv::Mat
	//int
	cv::Mat m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC1);
	grid_map=m_temp.clone();
	
	//float
	m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_32FC1);
	pot_map=m_temp.clone();
	grad_map[0]=m_temp.clone();
	grad_map[1]=m_temp.clone();
		
	//debug
	debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
	pub=nh_pub.advertise<sensor_msgs::Image>("pot_image",1);	
}
APF::APF()
	:max_pot(100),min_pot(-100){//,it(nh_pub){		
	//debug
	pub=nh_pub.advertise<sensor_msgs::Image>("pot_image",1);	
}
APF::~APF()
{
	grid_map.release();
	pot_map.release();
	grad_map[0].release();
	grad_map[1].release();
	debug_image.release();
}

