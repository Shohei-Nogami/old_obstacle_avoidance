#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>

#include"obst_avoid/point3d.h"
#include"obst_avoid/vel3d.h"
#include"obst_avoid/img_point.h"
#include"obst_avoid/points.h"
#include"obst_avoid/matching.h"
#include"obst_avoid/point2i.h"
#include"obst_avoid/cluster_element.h"
#include"obst_avoid/cluster.h"
#include"obst_avoid/cluster_point.h"
#include"obst_avoid/cluster_with_vel.h"
#include"time_class.h"
#include"image_class.h"
#include<fstream>//file input output
#include <Eigen/Dense>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>  
//#include <pcl/point_types.h> 
//#include <iostream>
class estimate_velocity{
	private:
		ros::NodeHandle nh_cluster,nh_optflw,nh_match,nh_pubpcl,nh_pub1,nh_pub2;

		ros::Publisher pc_pub,pub2;
		
		image_transport::ImageTransport it_pub1;
		image_transport::Publisher pub1;
		ros::Subscriber sub_cluster,sub_optflw,sub_match;

		ros::CallbackQueue queue_cluster,queue_optflw,queue_match;

		static const int width=672;
		static const int height=376;
		const float f=350.505;
		int ksize=3;
	//---cluster
		::obst_avoid::cluster pre_cluster;
		::obst_avoid::cluster cur_cluster;
		std::vector<int> cluster_match;
		std::vector<int> pre_cluster_match;
		//std::vector<std::vector<int> > pre_cluster_index;
		//std::vector<std::vector<int> > cur_cluster_index;
		//int **pre_cluster_index;
		//int **cur_cluster_index;
		int pre_cluster_index[height][width];
		int cur_cluster_index[height][width];
		std::vector<cv::Point3f> prepre_gp;
		std::vector<cv::Point3f> pre_gp;
		std::vector<cv::Point3f> cur_gp;
		std::vector<cv::Point3f> vel;
		std::vector<cv::Point3f> pre_vel;
		std::vector<cv::Point3f> prepre_vel;
		std::vector<cv::Point3f> acc;
		std::vector<cv::Point3f> pre_acc;
		
		std::vector<double> cur_cluster_size;
		std::vector<double> pre_cluster_size;
	//---opticalflow 
		::obst_avoid::matching match_msg;

	//--matching
		std::vector< std::vector<int> > match_n;
		std::vector< std::vector<float> > match_gp;
		std::vector<bool> matched;
		
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
	//debug	
		cv::Mat view_vel_image;
		cv::Mat view_image;
	//--time_cls
		time_class tm_cls;
		public:
			estimate_velocity();
			~estimate_velocity();


			void subsuctibe_cluster(void);		
			void cluster_callback(const obst_avoid::cluster::ConstPtr& msg);
			void subsuctibe_match_index(void);
			void match_index_callback(const obst_avoid::matching::ConstPtr& msg);
			bool matching_cluster(void);
			void set_gp(void);
			bool estimate_velocity_of_cluster(void);
			void predict_cluster(void);
			void draw_velocity(cv::Mat& image);
			void publish_pointcloud(void);
			cv::Mat& debug_image(cv::Mat& image);
			void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color);
			void publish_cluster_with_vel(void);
		
			void culc_distance_3f(const cv::Point3f x1,cv::Point3f x2,float& dis);
			void calmanfilter(void);
			void LPF(void);
			
			void record_odom_and_vel(void);
};
