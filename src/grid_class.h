#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

class grid_class{
	private:
		//pcl::PointCloud<pcl::PointXYZRGB> pcl_data;
		//pcl::PointCloud<pcl::PointXYZ> pcl_data;
		ros::NodeHandle nh_pub,nh_sub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
		cv::Mat grid_map;
		image_transport::ImageTransport it_pub;
		image_transport::Publisher pub;
		cv_bridge::CvImagePtr cvbridge_image;
		cv::Mat depth_image;
		int grid_resolution;//
		float grid_size;
		float grid_cell_size;
		
		const int width=672;
		const int height=376;
		const float f=350.505;
	public:
		grid_class();
		~grid_class();
		//void subscribe_pcl(void);
		void subscribe_depth_image(void);
		//void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void image_callback(const sensor_msgs::ImageConstPtr& msg);
		void set_grid_map(void);
		void publish_grid_map(void);
};

