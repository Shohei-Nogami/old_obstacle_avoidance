#include"img_prc_cls.h"

	void ImageProcesser::wheel_control(void){
		int vel_l;
		int vel_r;
		//deviation
		if(z_target<1){
			vel_l=0;
			vel_r=0;
		}
		else if(z_target==0.5){
			vel_l=0;
			vel_r=50;
		}
		else{
			vel_l=vel;
			vel_r=vel;
		}
		const double s=0.50;//sensitivity
//		std::cout<<"tp,wdth:"<<target_point.x<<","<<width/2<<"\n";
		double d=(target_point.x-width/2)/s;//+:roll right,-:roll left
		int u=(int)d;
		if(u>max_vel_dif)
			u=max_vel_dif;
		else if(u<-max_vel_dif)
			u=-max_vel_dif;
		vel_l=vel_l-u/2;
		vel_r=vel_r+u/2;
		wheelMsg.vel_l=vel_l;
		wheelMsg.vel_r=vel_r;

		if(std::abs(vel_l)>100){
			if(vel_l>0)
				wheelMsg.vel_data="0"+std::to_string(vel_l)+std::to_string(vel_r);
			else if(vel_l<0)
				wheelMsg.vel_data=std::to_string(vel_l)+"0"+std::to_string(vel_r);
		}
		else if(std::abs(vel_l)>10){
			if(vel_l>0)
				wheelMsg.vel_data="00"+std::to_string(vel_l)+"0"+std::to_string(vel_r);
			else if(vel_l<0)
				wheelMsg.vel_data="0"+std::to_string(vel_l)+"00"+std::to_string(vel_r);
		}
		else if(std::abs(vel_l)>0){
			if(vel_l>0)
				wheelMsg.vel_data="000"+std::to_string(vel_l)+"00"+std::to_string(vel_r);
			else if(vel_l<0)
				wheelMsg.vel_data="00"+std::to_string(vel_l)+"000"+std::to_string(vel_r);
		}
		else
			wheelMsg.vel_data="00000000";

//		std::cout<<"wheelMsg.vel_data:("<<wheelMsg.vel_data<<"\n";
		pub_wheel.publish(wheelMsg);
	}
