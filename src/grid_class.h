#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

class grid_class{
	private:
		//pcl::PointCloud<pcl::PointXYZRGB> pcl_data;
		pcl::PointCloud<pcl::PointXYZ> pcl_data;
		ros::NodeHandle nh_pub,nh_sub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
		cv::Mat grid_map;
		image_transport::ImageTransport it_pub;
		
		int grid_resolution;//
		float grid_size;
		float grid_cell_size;
	public:
		grid_class();
		~grid_class();
		void subscribe_pcl(void);
		void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void set_grid_map(void);
		void publish_grid_map(void);
};
