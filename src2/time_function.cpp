#include"img_prc_cls.h"
//----time----
	void ImageProcesser::gettime(void){
		ros::Duration time = ros::Time::now()-start_time;
		new_time=time.toSec();
	}
	void ImageProcesser::getprevtime(void){
		prev_time=new_time;
	}
	void ImageProcesser::culcdt(void){
		dt=new_time-prev_time;
	}
	void ImageProcesser::getimgtime(void){
		ros::Duration time = ros::Time::now()-start_time;
		new_img_time=time.toSec();
	}
	void ImageProcesser::getprevimgtime(void){
		prev_img_time=new_img_time;
	}
	void ImageProcesser::culcimgdt(void){
		img_dt=new_img_time-prev_img_time;
	}
