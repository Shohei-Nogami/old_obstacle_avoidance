#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Empty.h>

#include"struct_index.h"
#include"opticalflow_data.h"

#include"obst_avoid/point3d.h"
#include"obst_avoid/vel3d.h"
#include"obst_avoid/img_point.h"
#include"obst_avoid/points.h"
#include"obst_avoid/matching.h"
#include"time_class.h"
#include"image_class.h"
#include"obst_avoid/point2i.h"
#include"obst_avoid/cluster_element.h"
#include"obst_avoid/cluster.h"

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>  
//#include <pcl/point_types.h> 
//#include <iostream>
class detect_objects{
	private:
		ros::NodeHandle nh_pub1,nh_pub2,nh_pub3,nh_pub4;
		ros::NodeHandle nh_sub,nh_sub2;
		ros::NodeHandle nh_pubpcl;
		ros::Publisher pub_empty,pub_cluster;
		ros::Subscriber sub,sub2;
		ros::CallbackQueue queue;
		image_transport::ImageTransport it_pub1,it_pub2;
		image_transport::Publisher pub1,pub2;

		bool EXECUTED_CALLBACK;

		static const int width=672;
		static const int height=376;
		const float f=350.505;
		
		cv_bridge::CvImagePtr cvbridge_image;
		cv::Mat depth_image;

	//-----PCL
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  	ros::Publisher pc_pub;
		
		pcl::SACSegmentation<pcl::PointXYZ> seg;//の傾きラジアン
		pcl::PointIndices::Ptr inliers;// (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients;// (new pcl::ModelCoefficients);


	//--estimate velocity with gp
		std::vector<pcl::PointXYZ> cur_gp;
		std::vector<pcl::PointXYZ> pre_gp;
		std::vector<pcl::PointXYZ> cluster_vel_gp;
	
	//--density clustering
		//std::vector< std::vector<cv::Point2i> > Q;
		::obst_avoid::cluster Q;
		cv::Mat temp_image;
		cv::Mat filted_image;
		const int ksize=3;
		const int median_param=3;
	//--time_cls
		time_class tm_cls;
	//--response msg
		std_msgs::Empty emptymsg;
	public:
		detect_objects();
		~detect_objects();
		void subscribe_depth_image(void);
		void image_callback(const sensor_msgs::ImageConstPtr& msg);
		bool is_cvbridge_image(void);
		void set_depth_image(void);
	//---DENSITY BASED COLUSTERING
		void density_based_clustering(cv::Mat& image);
		cv::Mat& draw_cluster(cv::Mat& image);
		void filter_process(void);

	//---GROUND ESTIMATE
		void ground_estimation_from_image(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d);
		
	//---RESPONSE CALLBACK
		void empty_callback(const std_msgs::Empty& msg);
	//---publish_cluster
		void publish_cluster(void);
	//---publish response
		void publish_response(void);
};

