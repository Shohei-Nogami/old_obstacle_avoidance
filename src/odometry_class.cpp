#include"odometry_class.h"

odometry_class::odometry_class()
	:first_process_flag(true),first_delta_process_flag(true)
	,dx(0),dy(0),dz(0),droll(0),dpitch(0),dyaw(0)
{

}
bool odometry_class::is_cur_odometry(void){
	if(first_process_flag)
		return false;
	else
		return true;
}
bool odometry_class::is_delta_odometry(void){
	if(first_delta_process_flag)
		return false;
	else
		return true;
}
void odometry_class::turn_first_process_flag(void){
	first_process_flag=false;
}
void odometry_class::turn_first_delta_process_flag(void){
	first_delta_process_flag=false;
}

void odometry_class::subscribe_odometry(void){
	queue.callOne(ros::WallDuration(1));
}
//		void set_cur_image(void);
void odometry_class::set_odometry(void){
	if(is_cur_odometry()){
		set_pre_odometry();
		turn_first_process_flag();
	}
	subscribe_odometry();
}
void odometry_class::get_cur_odometry(double& x,double& y,double& z,double& r,double& p,double& yw){
	x=position_x;
	y=position_y;
	z=position_z;
	r=roll;
	p=pitch;
	yw=yaw;

}
void odometry_class::get_pre_odometry(double& x,double& y,double& z,double& r,double& p,double& yw){
	x=pre_position_x;
	y=pre_position_y;
	z=pre_position_z;
	r=pre_roll;
	p=pre_pitch;
	yw=pre_yaw;
}

void odometry_class::get_delta_odometry(double& return_dx,double& return_dz,double& return_dyaw){
	return_dx=dx;
	return_dz=dz;
	return_dyaw=dyaw;
}
odometry_class::~odometry_class(){
}
double& odometry_class::get_delta_yaw(void){
	return dyaw;
}
void odometry_class::set_pre_odometry(void){
	pre_position_x=position_x;
	pre_position_y=position_y;
	pre_position_z=position_z;
	pre_roll=roll;
	pre_pitch=pitch;
	pre_yaw=yaw;
}
void odometry_class::set_delta_orientetion(void){
	if(pre_yaw>0&&yaw<0&&pre_yaw>PI/2)
		dyaw=(yaw+2*PI)-pre_yaw;
	else if(pre_yaw<0&&yaw>0&&yaw>PI/2)
		dyaw=(yaw-2*PI)-pre_yaw;
	else
		dyaw=yaw-pre_yaw;
	droll=roll-pre_roll;
	dpitch=pitch-pre_pitch;
}
void odometry_class::define_variable(void){
	pub=nh_pub.advertise<nav_msgs::Odometry>("odometry",1);
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/odom",1,&odometry_class::odometry_callback,this);
}
void odometry_class::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg){
	//現在のodometryを取得
	double r,p,y;//一時的に格納するための変数
	tf::Quaternion quat(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf::Matrix3x3(quat).getRPY(r,p,y);
	//set
	position_x=msg->pose.pose.position.x;
	position_y=msg->pose.pose.position.y;
	position_z=msg->pose.pose.position.z;
	roll=r;
	pitch=p;
	yaw=y;

	position_x = ( cos(pitch)*cos(yaw) )*msg->pose.pose.position.x
	- ( cos(pitch)*sin(yaw) )*msg->pose.pose.position.y
	+ ( sin(pitch) )*msg->pose.pose.position.z;

	position_y = ( cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw) )*msg->pose.pose.position.x
	+ ( cos(roll)*cos(yaw) - sin(roll)*sin(pitch)*sin(yaw) )*msg->pose.pose.position.y
	- ( sin(roll)*cos(pitch) )*msg->pose.pose.position.z;

	position_z = ( sin(roll)*sin(yaw) - cos(roll)*sin(pitch)*cos(yaw) )*msg->pose.pose.position.x
	+ ( sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw) )*msg->pose.pose.position.y;
	+ ( cos(roll)*cos(pitch) )*msg->pose.pose.position.z;
}
void odometry_class::set_pre_delta_odometry(void){
	pre_dx=dx;
	pre_dz=dz;
	pre_droll=droll;
	pre_dpitch=dpitch;
	pre_dyaw=dyaw;
}
void odometry_class::set_delta_position(void){
	dr=sqrt((position_x*position_x
		-2*position_x*pre_position_x
		+pre_position_x*pre_position_x)
		+(position_y*position_y
		-2*position_y*pre_position_y
		+pre_position_y*pre_position_y)
		+(position_z*position_z
		-2*position_z*pre_position_z
		+pre_position_z*pre_position_z));
	dz=-dr*cos(-dyaw);
	dx=dr*sin(-dyaw);
/*
	double T=dt*5;
	dz=(T*pre_dz+dt*dz)/(T+dt);
	dx=(T*pre_dx+dt*dx)/(T+dt);
*/
	if(!pose_detection())
		dz=-dz;
}
//進行の向きを取得
bool odometry_class::pose_detection()
{
	bool status=true;
	if(-pre_yaw<(PI/2.0)&&-pre_yaw>0&&
		(-(position_y-pre_position_y)>0||(position_x-pre_position_x)>0)){
		if((-pre_yaw)-atan(
			(position_x-pre_position_x)/(-(position_y-pre_position_y)))<PI/2.0)
			status=false;
	}
	if(-pre_yaw>-(PI/2.0)&&-pre_yaw<0&&
		(-(position_y-pre_position_y)<0||(position_x-pre_position_x)>0)){
		if(-(-pre_yaw)+atan(
			(position_x-pre_position_x)/(-(position_y-pre_position_y)))<PI/2.0)
			status=false;
	}
	if(-pre_yaw>-(PI)&&-pre_yaw<-(PI/2.0)&&
		(-(position_y-pre_position_y)<0||(position_x-pre_position_x)<0)){
		if(PI+(-pre_yaw)-atan(
			(position_x-pre_position_x)/(-(position_y-pre_position_y)))<PI/2.0)
			status=false;
	}
	if(-pre_yaw<(PI)&&-pre_yaw>(PI/2.0)&&
	(-(position_y-pre_position_y)>0||(position_x-pre_position_x)<0)){
		if(PI-(-pre_yaw)+atan(
			(position_x-pre_position_x)/(-(position_y-pre_position_y)))<PI/2.0)
			status=false;
	}
	if(-pre_yaw==0&&(position_x-pre_position_x)>0)
		status=false;

	if(-pre_yaw==PI/2.0&&-(position_y-pre_position_y)>0)
		status=false;

	if(-pre_yaw==-PI/2.0&&-(position_y-pre_position_y)<0)
		status=false;

	if((-pre_yaw==-PI||-pre_yaw==PI)&&(position_x-pre_position_x)<0)
		status=false;
	return status;
}
void odometry_class::set_delta_odometry(void){
	if(is_delta_odometry()){
		set_pre_delta_odometry();
		turn_first_delta_process_flag();
	}
	set_delta_orientetion();
	set_delta_position();
}
/*
int main(int argc,char **argv){
	ros::init(argc,argv,"odometry_class_test");
	odometry_class odometry;
	std::cout<<"finish\n";
	return 0;
}
*/

