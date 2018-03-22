#include"ros/ros.h"
#include <ros/callback_queue.h>

class time_class{
	private:
		double cur_time,pre_time;
		double delta_time;
		ros::Duration temp_time;
		ros::Time start_time;
		bool first_process_flag;
		ros::NodeHandle nh;
	public:
		time_class();
		~time_class();
		void set_cur_time(void);
		void set_pre_time(void);
		void set_delta_time(void);
		void get_delta_time(double& dt);
};
