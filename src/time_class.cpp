#include"time_class.h"

time_class::time_class()
	:first_process_flag(true)
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
void time_class::get_delta_time(double& dt){
	dt=delta_time;
}


int main(int argc,char **argv){
	ros::init(argc,argv,"time_class_test");
	time_class time;
	std::cout<<"finish\n";
	return 0;
}

