#include"odometry_class.h"

odometry_class::odometry_class()
	:x(0),y(0),z(0),r(0),p(0),yw(0),wv(0),ww(0),vv(0),vw(0),th_angle(M_PI/18)
{
	wocls.define_variable();
	vocls.define_variable();
	
}
odometry_class::~odometry_class(){
}
void odometry_class::subscribe_msgs(void){
	wocls.subscribe_odometry();
	vocls.set_odometry();
}
void odometry_class::set_data(void){
	wocls.set_current_velocity();
	wocls.set_velocity_X();
	vocls.set_delta_odometry();
	vocls.set_velocity();
}
void odometry_class::set_velocity(void){
	//set velocity X
	vv_x=vocls.get_velocity_x();
	vv_y=vocls.get_velocity_y();
	vv_z=vocls.get_velocity_z();
	wv_x=wocls.get_velocity_x();
	wv_y=wocls.get_velocity_y();
	wv_z=wocls.get_velocity_z();

	//inner product
	ip=vv_x*wv_x+vv_y*wv_y+vv_z*wv_z;
	
	//length of vector
	vv_size=std::sqrt(vv_x*vv_x+vv_y*vv_y+vv_z*vv_z);
	wv_size=std::sqrt(wv_x*wv_x+wv_y*wv_y+wv_z*wv_z);
	
	//difference angle of vectors
	dif_angle=ip/(vv_size*wv_size);
	
	//angle => th_angle -> v=wv
	//angle = 0 -> v=vv no-use
	//eq:y=e^(-X^2)
	//y=0.5 -> angle==th_angle/2
	double P=exp(0.8326*( dif_angle/(th_angle/2) ) );			
	
	vv=vocls.get_velocity();
	vw=vocls.get_velocity_wy();
	
	wv=wocls.get_velocity();
	ww=wocls.get_angular_velocity();
	
	v=vv*P+wv*(1-P);
	w=vw*P+ww*(1-P);
	
}
void odometry_class::set_odometry(void){
	tm_odm.set_time();
	x+=v*sin(-w*tm_odm.get_delta_time());
	y+=0;
	z+=v*cos(-w*tm_odm.get_delta_time());
	r+=0;
	p+=0;
	yw+=w*tm_odm.get_delta_time();
}
double& odometry_class::get_dif_angle(void){
	return dif_angle;
}
double& odometry_class::get_angular_velocity(void){
	return w;
}
double& odometry_class::get_velocity(void){
	return v;
}
double& odometry_class::get_odometry_x(void){
	return x;
}
double& odometry_class::get_odometry_y(void){
	return y;
}
double& odometry_class::get_odometry_z(void){
	return z;
}
double& odometry_class::get_odometry_r(void){
	return r;
}
double& odometry_class::get_odometry_p(void){
	return p;
}
double& odometry_class::get_odometry_yw(void){
	return yw;
}


int main(int argc,char **argv){
	ros::init(argc,argv,"odometry_class_test");
	odometry_class odom_cls;
	time_class t;
	double t_now=t.get_time_now();
	while(ros::ok()){
		odom_cls.subscribe_msgs();
		std::cout<<"subscribed:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		odom_cls.set_data();
		std::cout<<"set_data:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		odom_cls.set_velocity();
		std::cout<<"set_velocity:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		odom_cls.set_odometry();
		std::cout<<"set_odometry:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		
		t.set_time();
		std::cout<<"dt:"<<t.get_delta_time()<<"\n";
	}
	
	std::cout<<"finish\n";
	return 0;
}
