#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"obst_avoid/point3d.h"
#include"obst_avoid/object_info.h"
#include"obst_avoid/objects_info.h"
#include"time_class.h"
#include <pcl_ros/point_cloud.h>
//#include<Eigen/Core>
//#include<Eigen/Dense>
#include<fstream>//file input output

class estimate_velocity{
	private:
		ros::NodeHandle nh_sub,nh_pub,nh_pcl;
		ros::Publisher pub,pub_pcl;
		ros::Subscriber sub;
		ros::CallbackQueue queue;

		obst_avoid::objects_info cur_objs;
		obst_avoid::object_info obj;

		obst_avoid::objects_info pre_objs;
		obst_avoid::objects_info prepre_objs;


		std::vector<cv::Point3f> vel;
		std::vector<cv::Point3f> pre_vel;
		std::vector<cv::Point3f> prepre_vel;

		std::vector<cv::Point3f> acc;
		std::vector<cv::Point3f> pre_acc;


		
		//----tracking count
		std::vector<int> track_n;
		std::vector<int> pre_track_n;

		
		//---calmanfilter
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > xh_t;
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > sig_xh_t;
	
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > xh_t_1;
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > sig_xh_t_1;
	
		Eigen::MatrixXd sig_ut;
		Eigen::MatrixXd del_t;
		Eigen::MatrixXd sig_x0;
		Eigen::MatrixXd I;

		time_class tm_cls;


		//write files
		std::string ofilename;
	public:
		estimate_velocity();
		~estimate_velocity();

		void subscribe_objects(void);
		void objects_callback(const obst_avoid::objects_info::ConstPtr& msg);
		bool culculate_velocity(void);
		void culculate_accelerate(void);
	
		void calmanfilter(void);

		void LPF(float& x_t,const float& x_t_1,double& dt,const double& T);
		//void estimate_velocity(void);
		void culc_distance_3f(const cv::Point3f& x1,cv::Point3f& x2,float& dis);
		void record_odom_and_vel(void);
		void publish_pointcloud(void);
};

