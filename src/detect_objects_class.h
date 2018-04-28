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

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>  
//#include <pcl/point_types.h> 
//#include <iostream>
class detect_objects{
	private:
		ros::NodeHandle nh_pub1,nh_pub2,nh_sub;
		ros::NodeHandle nh_pubpcl,nh_pubpcl2,nh_pubpcl3,nh_pubpcl4,nh_pubpcl5;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
		image_transport::ImageTransport it_pub1,it_pub2;
		image_transport::Publisher pub1,pub2;



		static const int width=672;
		static const int height=376;
		const float f=350.505;
		
		bool EXECUTED_CALLBACK;
		cv_bridge::CvImagePtr cvbridge_image;
		cv::Mat depth_image;
		//DEM
		const double map_size_z=16.04; //[m]
		const double map_size_x=8.04;  //[m]
		static const int map_size_nz=401;  //map height [pixcel]
		static const int map_size_nx=201;  //map width  [pixcel]
		const double cell_size=0.04;//[cm]
		const double h_th=0.08;
		std::vector<double> dem_element[map_size_nz][map_size_nx];
		std::vector<cv::Point2f> dem_cluster[map_size_nz][map_size_nx];//x:low,y:high
		
		struct index_schema{
			int nx;//index to number of x in dem-map
			int nz;//index to number of z in dem-map
			float y;
			int ny;//index to shema cluster
			index_schema() : nz(-1),nx(-1),y(-1),ny(-1){}
		};
		struct index_image{
			int h;//height of image
			int w;//width of image
			float y;
		};
		index_schema index_schm[height][width];
		std::vector< index_image > index_img[map_size_nz][map_size_nx];

		//PCL
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_deleted_cloud;
		//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Eclusted_cloud;
  	ros::Publisher pc_pub,pc_pub2,pc_pub3,pc_pub4,pc_pub5;
		
		pcl::SACSegmentation<pcl::PointXYZ> seg;//の傾きラジアン
		pcl::PointIndices::Ptr inliers;// (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients;// (new pcl::ModelCoefficients);
	
		public:
			detect_objects();
			~detect_objects();
			void subscribe_depth_image(void);
			void image_callback(const sensor_msgs::ImageConstPtr& msg);
			bool is_cvbridge_image(void);
			void set_depth_image(void);
			bool convert_coordinate_xz_nxz(const float x,const float z,int& nx,int& nz);
			void invert_coordinate_xz_nxz(const int& nx,const int& nz,float& x,float& z);
			void convet_image_to_pcl(cv::Mat& image);

			void ground_estimation_from_pc(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d);
			void ground_estimation_from_image(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d);

			void set_DEM_map(void);	
			void clustering_DEM_elements(void);
			void clustering_schema(void);
			void clustering_slice(void);
		
			void convert_dem_to_pcl(void);
			void clear_dem_element(void);
};
