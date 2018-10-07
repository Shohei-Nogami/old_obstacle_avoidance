#include"time_class.h"

time_class::time_class()
	:first_process_flag(true),delta_time_d(0)
{
	start_time = ros::Time::now();
}
time_class::~time_class()
{
}

void time_class::set_cur_time(void){
	//temp_time = ros::Time::now()-start_time;
	//cur_time=temp_time.toSec();
	cur_time=ros::Time::now();
	first_process_flag=false;
}
void time_class::set_pre_time(void){
	pre_time=cur_time;
}
void time_class::set_delta_time(void){
	delta_time=cur_time-pre_time;
}
double& time_class::get_delta_time(/*double& dt*/void){
	// dt=delta_time;
	//return delta_time;
	delta_time_d=delta_time.toSec();
	return delta_time_d;
}
double& time_class::get_time_now(void){
	temp_time = ros::Time::now()-start_time;
	nowtime=temp_time.toSec();
	return nowtime;
}
void time_class::set_time(void){
	if(!first_process_flag){
		set_pre_time();
	}
	else{
		set_cur_time();
		set_pre_time();
	}
	set_cur_time();
	set_delta_time();
}
bool time_class::is_delta_time(void){
	if(delta_time_d){
		return true;
	}
	else{
		return false;
	}
}
/*
int main(int argc,char **argv){
	ros::init(argc,argv,"time_class_test");
	time_class time;
	std::cout<<"finish\n";
	return 0;
}
*/
