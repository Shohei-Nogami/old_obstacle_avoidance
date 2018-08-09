
#ifndef INCLUDE_VISUAL_ODOMETRY_CLASS
#define INCLUDE_VISUAL_ODOMETRY_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
//odometry取得用
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//ros msgファイル
#include<nav_msgs/Odometry.h>

//#include<obst_avoid/src/tm_cls/time_class.h>
#include"time_class.h"
//円周率
#define PI 3.141593

class visual_odometry_class{
	private:
		double position_x,position_y,position_z;
		double pre_position_x,pre_position_y,pre_position_z;
		double global_dx,global_dy,global_dz;
		double roll,pitch,yaw;
		double pre_roll,pre_pitch,pre_yaw;
		double droll,dpitch,dyaw;
		double pre_dx,pre_dz,pre_droll,pre_dpitch,pre_dyaw;
		double dr,pre_dr;
		double dx,dy,dz;
		double dt,pre_dt;
		double vx,vy,vz,wy,v;
		bool first_process_flag;
		bool first_delta_process_flag;
		//time_class tm_vsodm;
		ros::Time cur_time;
		ros::Time pre_time;
		ros::Duration delta_time;
	public:
		visual_odometry_class();
		bool is_cur_odometry(void);
		bool is_delta_odometry(void);
		void turn_first_process_flag(void);
		void turn_first_delta_process_flag(void);
		void subscribe_odometry(void);
		void set_odometry(void);//<-use
		void get_cur_odometry(double& x,double& y,double& z,double& r,double& p,double& yw);//<-use
		void get_pre_odometry(double& x,double& y,double& z,double& r,double& p,double& yw);//<-use
		void get_delta_odometry(double& return_dx,double& return_dz,double& return_dyaw);//<-use
		void set_velocity(void);
		double& get_velocity_x(void);
		double& get_velocity_y(void);
		double& get_velocity_z(void);
		double& get_velocity_wy(void);
		double& get_velocity(void);
		double get_delta_time(void);
		
		virtual ~visual_odometry_class();
		double& get_delta_yaw(void);
		void set_delta_odometry(void);//<-use
		virtual void define_variable(void);

	protected:
		ros::NodeHandle nh_pub,nh_sub;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;

		virtual void set_pre_odometry(void);
		virtual void set_delta_orientetion(void);
		virtual void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
		void set_pre_delta_odometry(void);
		void set_delta_position(void);
		bool pose_detection(void);
		bool set_dt(void);

//		virtual void publish_debug_image(void);


/*
	void ImageProcesser::wheelodom_callback(const obst_avoid::wheel_msg::ConstPtr& msg){
		vr=msg->vel_r;
		vl=msg->vel_l;
	}
	void ImageProcesser::setwodom(void){
		wodom_queue.callOne(ros::WallDuration(0.001));
	}
*/


};

#endif
