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
#include"time_class.h"
#include"image_class.h"

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>  
//#include <pcl/point_types.h> 
//#include <iostream>
class estimate_velocity{
	private:
		ros::NodeHandle nh_cluster,nh_optflw,nh_match,nh_pubpcl,nh_pub1;

		ros::Publisher pc_pub;
		
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
		std::vector<cv::Point3f> pre_gp;
		std::vector<cv::Point3f> cur_gp;
		std::vector<cv::Point3f> vel;
		std::vector<cv::Point3f> pre_vel;
	//---opticalflow 
		::obst_avoid::matching match_msg;

	//--matching
		std::vector< std::vector<int> > match_n;
		std::vector<bool> matched;
		//in recently github
	
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
			bool estimate_velocity_of_cluster(void);
			void draw_velocity(cv::Mat& image);
			void publish_pointcloud(void);
			cv::Mat& debug_image(cv::Mat& image);
			void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color);

};
