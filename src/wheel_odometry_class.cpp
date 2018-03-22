#include"wheel_odometry_class.h"

wheel_odometry_class::wheel_odometry_class()
	:first_process_flag(true),first_delta_process_flag(true)
	,position_x(0),position_z(0),vr(0),vl(0)
{
}
bool wheel_odometry_class::is_cur_odometry(void){
	if(first_process_flag)
		return false;
	else 
		return true;
}
bool wheel_odometry_class::is_delta_odometry(void){
	if(first_delta_process_flag)
		return false;
	else 
		return true;
}
void wheel_odometry_class::turn_first_process_flag(void){
	first_process_flag=false;
}
void wheel_odometry_class::turn_first_delta_process_flag(void){
	first_delta_process_flag=false;
}
void wheel_odometry_class::subscribe_odometry(void){
	queue.callOne(ros::WallDuration(0.001));
}
void wheel_odometry_class::set_wheel_odometry(void){//<-use
	position_x+=dx;
	position_z+=dz;
	yaw+=dyaw;
}
void wheel_odometry_class::get_cur_odometry(double& x,double& z,double& yw){//<-use
//on hold
}
void wheel_odometry_class::get_pre_odometry(double& x,double& z,double& yw){//<-use
//on hold
}
void wheel_odometry_class::get_delta_odometry(double& x,double& z,double& yw){//<-use
//on hold
}
wheel_odometry_class::~wheel_odometry_class(){
}

void wheel_odometry_class::set_velocity_and_angular_velocity(void){
	v=(double)(vr+vl)/2/1000;
	w=(double)(vr-vl)/d/1000;
}

void wheel_odometry_class::set_cur_odometry(void){
	//on hold
}
void wheel_odometry_class::set_pre_odometry(void){
	pre_position_x=position_x;
	pre_position_z=position_z;
}
void wheel_odometry_class::set_delta_position(double& dt){
	dx=v*dt*sin(dyaw);
	dz=v*dt*cos(dyaw);
}
void wheel_odometry_class::set_delta_orientetion(double& dt){
	dyaw=w*dt;
}
void wheel_odometry_class::define_variable(void){	
	pub=nh_pub.advertise<obst_avoid::wheel_msg>("wheel_odometry",1);
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/wheel_data",1,&wheel_odometry_class::wheel_odometry_callback,this);
}
void wheel_odometry_class::wheel_odometry_callback(const obst_avoid::wheel_msg::ConstPtr& msg){
	vr=msg->vel_r;
	vl=msg->vel_l;
}
void wheel_odometry_class::set_pre_delta_odometry(void){
	pre_dx=dx;
	pre_dz=dz;
	pre_dyaw=dyaw;
}
void wheel_odometry_class::set_delta_odometry(double& dt){//<-use
	if(is_delta_odometry()){
		set_pre_delta_odometry();
		turn_first_delta_process_flag();
	}
	set_delta_orientetion(dt);
	set_delta_position(dt);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"wheel_odometry_class_test");
	wheel_odometry_class wheel_odometry;
	std::cout<<"finish\n";
	return 0;
}



