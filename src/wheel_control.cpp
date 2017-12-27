#include"img_prc_cls.h"
	

	void ImageProcesser::image_based_travel(void){
		int vel_r;
		int vel_l;
//		std::cout<<"target_point.x:"<<target_point.x<<"\n";
//		std::cout<<"vel:"<<vel<<"\n";
		double T=dt*5;
//		if(PRD_PRC_ORDER==MOVING_SLOWLY)
//			std::cout<<"target_vel:"<<target_vel<<"\n";
		if(z_target<0.7&&PRD_PRC_ORDER!=LOCATION_BASED_TRAVEL){
			target_vel=0:
		}
		vel=(T*pvel+dt*target_vel)/(T+dt);
		pvel=vel;
		vel_r=vel;
		vel_l=vel;
		const double s=0.5;//sensitivity
		double d=(target_point.x-width/2)/s;//+:roll right,-:roll left
		int u=(int)d;
		if(u>max_vel_dif)
			u=max_vel_dif;
		else if(u<-max_vel_dif)
			u=-max_vel_dif;
//		std::cout<<"max_vel_dif:"<<max_vel_dif<<"\n";
//		std::cout<<"u:"<<u<<"\n";
		vel_r=vel_r-u/2;
		vel_l=vel_l+u/2;
//		vel_r=(T*pvel_r+dt*vel_r)/(T+dt);
//		vel_l=(T*pvel_l+dt*vel_l)/(T+dt);
//		pvel_r=vel_r;
//		pvel_l=vel_l;		
//		std::cout<<"vel_l,vel_r:"<<vel_l<<","<<vel_r<<"\n";
		wheelMsg.vel_r=vel_r;
		wheelMsg.vel_l=vel_l;
//		std::cout<<"wheelMsg:"<<wheelMsg<<"\n";
//		if(PRD_PRC_ORDER==MOVING_SLOWLY)
//			std::cout<<"wheelMsg:"<<wheelMsg<<"\n";
	}

	void ImageProcesser::wheel_control(void){

//		cv::circle(Limg_view, target_point, 4, cv::Scalar(200,200,0),-1, CV_AA);


		if(z_target<0.7&&PRD_PRC_ORDER!=MOVING_SLOWLY&&PRD_PRC_ORDER!=LOCATION_BASED_TRAVEL){

			double T=dt*5;
			if(target_point.x==24){
				wheelMsg.vel_r=-100;
				wheelMsg.vel_l=0;				
//				wheelMsg.vel_r=(T*pvel+dt*(-100))/(T+dt);
//				wheelMsg.vel_l=(T*pvel+dt*0)/(T+dt);				
			}
			else if(target_point.x==0){
				wheelMsg.vel_r=0;
				wheelMsg.vel_l=-100;					
//				wheelMsg.vel_r=(T*pvel+dt*0)/(T+dt);			
//				wheelMsg.vel_l=(T*pvel+dt*(-100))/(T+dt);				
			}
			else{
				wheelMsg.vel_r=-100;
				wheelMsg.vel_l=0;
//				wheelMsg.vel_r=(T*pvel+dt*(-100))/(T+dt);
//				wheelMsg.vel_l=(T*pvel+dt*0)/(T+dt);				
			}
			pvel=0;

//			vel_r=-100;
//			vel_l=0;
		}
		
/*		if(wheelMsg.vel_l==0||wheelMsg.vel_r==0){
			if(wheelMsg.vel_l==0)
				wheelMsg.vel_data="0000"+std::to_string(wheelMsg.vel_r);
			else
				wheelMsg.vel_data=std::to_string(wheelMsg.vel_l)+"0000";
		}
		else{
			if(std::abs(wheelMsg.vel_l)>=100){
				if(wheelMsg.vel_l>0)
					wheelMsg.vel_data="0"+std::to_string(wheelMsg.vel_l)+std::to_string(wheelMsg.vel_r);
				else if(wheelMsg.vel_l<0)
					wheelMsg.vel_data=std::to_string(wheelMsg.vel_l)+"0"+std::to_string(wheelMsg.vel_r);
			}
			else if(std::abs(wheelMsg.vel_l)>=10){
				if(wheelMsg.vel_l>0)
					wheelMsg.vel_data="00"+std::to_string(wheelMsg.vel_l)+"0"+std::to_string(wheelMsg.vel_r);
				else if(wheelMsg.vel_l<0)
					wheelMsg.vel_data="0"+std::to_string(wheelMsg.vel_l)+"00"+std::to_string(wheelMsg.vel_r);
			}
			else if(std::abs(wheelMsg.vel_l)>0){
				if(wheelMsg.vel_l>0)
					wheelMsg.vel_data="000"+std::to_string(wheelMsg.vel_l)+"00"+std::to_string(wheelMsg.vel_r);
				else if(wheelMsg.vel_l<0)
					wheelMsg.vel_data="00"+std::to_string(wheelMsg.vel_l)+"000"+std::to_string(wheelMsg.vel_r);
			}
			else 
				wheelMsg.vel_data="00000000";
		}
*/
 		std::string order_l,order_r;
		if(wheelMsg.vel_l<0){
			if(wheelMsg.vel_l>-100){
				if(wheelMsg.vel_l>-10)
					order_l="00"+std::to_string(wheelMsg.vel_l);
				else
					order_l="0"+std::to_string(wheelMsg.vel_l);
			}
			else
				order_l=std::to_string(wheelMsg.vel_l);
		}
		else if(wheelMsg.vel_l>0){
			if(wheelMsg.vel_l<100)
				order_l="00"+std::to_string(wheelMsg.vel_l);
			else
				order_l="0"+std::to_string(wheelMsg.vel_l);
		}
		else
			order_l="0000";
		
		if(wheelMsg.vel_r<0){
			if(wheelMsg.vel_r>-100){
				if(wheelMsg.vel_r>-10)
					order_r="00"+std::to_string(wheelMsg.vel_r);				
				else
					order_r="0"+std::to_string(wheelMsg.vel_r);				
			}
			else
				order_r=std::to_string(wheelMsg.vel_r);
		}
		else if(wheelMsg.vel_r>0){
			if(wheelMsg.vel_r<100)
				order_r="00"+std::to_string(wheelMsg.vel_r);
			else
				order_r="0"+std::to_string(wheelMsg.vel_r);
		}
		else
			order_r="0000";
		wheelMsg.vel_data=order_l+order_r;
		
//		std::cout<<"wheelMsg.vel_data:("<<wheelMsg.vel_data<<"\n";
		pub_wheel.publish(wheelMsg);
	}


