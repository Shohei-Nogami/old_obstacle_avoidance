#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>
//#include <pcl_ros/point_cloud.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

class grid_class{
	private:
		//pcl::PointCloud<pcl::PointXYZRGB> pcl_data;
		//pcl::PointCloud<pcl::PointXYZ> pcl_data;
		ros::NodeHandle nh_pub,nh_pub2,nh_sub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
		cv::Mat grid_map;
		cv::Mat grid_map_view,binary_grid_map_view;
		image_transport::ImageTransport it_pub;
		image_transport::Publisher pub;
		image_transport::ImageTransport it_pub2;
		image_transport::Publisher pub2;
		cv_bridge::CvImagePtr cvbridge_image;
		cv::Mat depth_image;
		int grid_resolution;//
		float grid_size;
		float grid_cell_size;
		
		const int width=672;
		const int height=376;
		const float f=350.505;
		
		bool EXECUTED_CALLBACK;
		uint8_t/*uchar*/ binary_threshold;

//parameter of trajectory
		const int vel_patern=9;
		const float temp_v=0.2;
		const float d=0.138;
//		float temp_vl[vel_patern],temp_vr[vel_patern];
		std::vector<float> temp_vl,temp_vr;
		std::vector<double> temp_w;
		std::vector<double> temp_p;
		std::vector<double> delta_theta;
		std::vector<int> max_process_n;
		std::vector<int> rank_trajectory;
//parameter of collision avoidanace 
		float R=0.2;
		float d_r=0.1;
		int jn_Rd;
		std::vector<int> in_Rd;
		
	public:
		grid_class();
		~grid_class();
		//void subscribe_pcl(void);
		void subscribe_depth_image(void);
		//void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void image_callback(const sensor_msgs::ImageConstPtr& msg);
		bool is_cvbridge_image(void);
		void set_grid_map(void);
		void select_best_trajectory(void);
		void transport_robot_to_gridn(const double& xr,const double& yr,int& n_xr,int& n_yr);
		void transport_gridx_to_gridn(const double& x,const double& y,int& n_x,int& n_y);
		bool is_obstacle(const int nx,const int ny);
		void draw_best_trajectory(const int& num);
		void set_grid_map_view(void);
		void set_binary_grid_map_view(void);
		void publish_grid_map_view(void);
		void publish_binary_grid_map_view(void);

		void draw_all_trajectory(void);
			
};

