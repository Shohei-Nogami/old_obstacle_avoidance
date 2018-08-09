
#ifndef INCLUDE_TIME_CLASS
#define INCLUDE_TIME_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>

class time_class{
	private:
		//double cur_time,pre_time;
		ros::Time cur_time,pre_time;
		//double delta_time;
		ros::Duration temp_time;
		ros::Duration delta_time;
		ros::Time start_time;
		bool first_process_flag;
		ros::NodeHandle nh;
		double nowtime;
		double delta_time_d;
	public:
		time_class();
		virtual ~time_class();
		void set_cur_time(void);
		void set_pre_time(void);
		void set_delta_time(void);
		// void get_delta_time(double& dt);
		double& get_delta_time(void);
		double& get_time_now(void);
		void set_time(void);
		bool is_delta_time(void);
};

#endif
