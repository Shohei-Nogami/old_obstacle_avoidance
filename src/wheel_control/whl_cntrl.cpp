#include"whl_cntrl.h"
	wheel_control_class::wheel_control_class()
	{
		pub_wheel=nh_pub.advertise<obst_avoid::wheel_msg>("wheel_data",1);
		nh_sub.setCallbackQueue(&queue);
		sub_vel=nh_sub.subscribe("/vel_data",1,&wheel_control_class::velocity_callback,this);
	}
	wheel_control_class::~wheel_control_class()
	{
	}

	void wheel_control_class::subscribe_wheel_velocity(void)
	{
		queue.callOne(ros::WallDuration(1));
	}

	void wheel_control_class::velocity_callback(const obst_avoid::wheel_msg::ConstPtr& msg)
	{
		wheelMsg.vel_l=msg->vel_l;
		wheelMsg.vel_r=msg->vel_r;
		
	}

	bool wheel_control_class::set_msg(void)
	{
		if(std::isnan(wheelMsg.vel_l))
		{
			return false;
		}
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
	}

	void wheel_control_class::publish_velocity(void)
	{
		pub_wheel.publish(wheelMsg);
	}

int main(int argc,char **argv){
	ros::init(argc,argv,"wheel_control_class_test");
	wheel_control_class wcc;
	std::cout<<"ready\n";

	while(ros::ok()){
		wcc.subscribe_wheel_velocity();
		if(wcc.set_msg())
		{
			wcc.publish_velocity();
		}	
		std::cout<<"processing\n";
	}

	return 0;
}




