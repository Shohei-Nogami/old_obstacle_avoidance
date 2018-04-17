#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>

class detect_objects{
	private:
		ros::NodeHandle nh_pub,nh_pub2,nh_sub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
		image_transport::ImageTransport it_pub1,it_pub2;
		image_transport::Publisher pub1,pub2;

		const double map_size_z=16.04; //[m]
		const double map_size_x=8.04;  //[m]
		const int map_size_nz=401;  //map height [pixcel]
		const int map_size_nx=201;  //map width  [pixcel]
		const double cell_size=0.04;//[cm]
		std::vector<double> dem_element[map_size_nz][map_size_nx];
		std::vector<cv::Point2f> dem_cluster[map_size_nz][map_size_nx];//x:low,y:high
		const h_th=0.08;

		const int width=672;
		const int height=376;
		const float f=350.505;
		
		bool EXECUTED_CALLBACK;

		cv::Mat depth_image;


		public:
			detect_objects();
			~detect_objects();
			void subscribe_depth_image(void);
			void image_callback(const sensor_msgs::ImageConstPtr& msg);
			bool is_cvbridge_image(void);
			void set_DEM_map(void);	
			void clustering_DEM_elements(void);
			void clustering_schema(void);
			void transrate_coordinate_xz_nxz(const double x,const double z,int& nx,int& nz);

};

