#include"time_class.h"

time_class::time_class()
	:first_process_flag(true),delta_time(0)
{
	start_time = ros::Time::now();
}
time_class::~time_class()
{
}
void time_class::set_cur_time(void){
	temp_time = ros::Time::now()-start_time;
	cur_time=temp_time.toSec();
}
void time_class::set_pre_time(void){
	pre_time=cur_time;
}
void time_class::set_delta_time(void){
	delta_time=cur_time-pre_time;
}
double& time_class::get_delta_time(/*double& dt*/void){
	// dt=delta_time;
	return delta_time;
}
void time_class::set_time(void){
	if(!first_process_flag){
		set_pre_time();
	}
	set_cur_time();
	set_delta_time();
}
/*
int main(int argc,char **argv){
	ros::init(argc,argv,"time_class_test");
	time_class time;
	std::cout<<"finish\n";
	return 0;
}
*/

