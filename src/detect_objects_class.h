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

#include"struct_index.h"
#include"opticalflow_data.h"

#include"obst_avoid/point3d.h"
#include"obst_avoid/vel3d.h"
#include"obst_avoid/img_point.h"
#include"time_class.h"
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>  
//#include <pcl/point_types.h> 
//#include <iostream>
class detect_objects{
	private:
		ros::NodeHandle nh_pub1,nh_pub2,nh_sub;
		ros::NodeHandle nh_pubpcl,nh_pubpcl2,nh_pubpcl3,nh_pubpcl4,nh_pubpcl5,nh_pubpcl6,nh_pubpcl7,nh_pubpcl8,nh_pubpcl9;
		ros::NodeHandle nh_optflw;
		ros::Subscriber sub,sub_optflw;
		ros::CallbackQueue queue,queue_optflw;
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
		const double map_size_y=1.5;   //[m]
		static const int map_size_nz=201;  //map height [pixcel]
		static const int map_size_nx=201;  //map width  [pixcel]
		static const int map_size_ny=38;  //map width  [pixcel]
		const double cell_size=0.05;//[cm]
		const double h_th=0.04;//0.08
		//std::vector<double> dem_element[map_size_nz][map_size_nx];//401*201
		std::vector<double> **dem_element;//[map_size_nz][map_size_nx];//401*201
		std::vector<cv::Point2f> dem_cluster[map_size_nz][map_size_nx];//x:low,y:high//401*201
		std::vector< std::vector<cv::Point2i> > *slice_cluster;
		std::vector<int> **slice_cluster_index;//[map_size_nz][map_size_nx];
		std::vector< std::vector<pcl::PointXYZ> > *slice_cluster_velocity_element;
		std::vector<pcl::PointXYZ> *slice_cluster_velocity;
		index_schema **index_schm;//673*376
		std::vector<index_image> **index_img;//[map_size_nz][map_size_nx];//401*201

	//-----PCL
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_deleted_cloud;
		//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Eclusted_cloud;
  	ros::Publisher pc_pub,pc_pub2,pc_pub3,pc_pub4,pc_pub5,pc_pub6,pc_pub7,pc_pub8,pc_pub9;
		
		pcl::SACSegmentation<pcl::PointXYZ> seg;//の傾きラジアン
		pcl::PointIndices::Ptr inliers;// (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients;// (new pcl::ModelCoefficients);
	//-----voxel grid
		index_voxel **index_vxl;//[height][width]
		std::vector<pcl::PointXYZ> ***voxel_element;
		//pcl::PointXYZ ***voxel_point;
		Point3f1i ***voxel_point;
		int ***index_points;
		pcl::PointCloud<pcl::PointXYZ>::Ptr selfvoxel_cloud;
		float voxel_size_x=0.04;
		float voxel_size_z=0.08;
		float voxel_size_y=0.04;

	//---clustering 
		int ***clusted_index;
		//std::vector< std::vector<pcl::PointXYZ> > cluster;
		std::vector< std::vector<Point3f1i> > cluster;
		std::vector<index_voxel> clst_tsk;
		
		//std::vector<pcl::PointXYZ> cluster_elements;
		//std::vector<cv::Point3i> cluster_elements_num;

	//---opticalflow
		::obst_avoid::vel3d vX;
	

	//--time_cls
		time_class tm_cls;
		public:
			detect_objects();
			~detect_objects();
			void subscribe_depth_image(void);
			void image_callback(const sensor_msgs::ImageConstPtr& msg);
			bool is_cvbridge_image(void);
			void set_depth_image(void);
			bool convert_coordinate_xz_nxz(const float x,const float z,int& nx,int& nz);
			
			void invert_coordinate_xz_nxz(const int& nx,const int& nz,float& x,float& z);
		//---SELF VOXEL CLUSTER
			void create_voxel_grid(cv::Mat& image);
			bool culc_voxel_nxzy(const float voxel_size_x,const float voxel_size_y,const float voxel_size_z,const float x,const float z,const float y,int& nx,int& nz,int& ny);
			bool convert_xzy_nxzy(const float& x,const float& z,const float& y,int& nx,int& nz,int& ny);
			float culclate_euclid_distance(pcl::PointXYZ& p1,pcl::PointXYZ& p2);
			float culclate_chebyshev_distance(pcl::PointXYZ& p1,pcl::PointXYZ& p2);
			float culclate_chebyshev_distance(Point3f1i& p1,Point3f1i& p2);
		//---OPTICALFLOW	
			void subscribe_opticalflow(void);
			void opticalflow_callback(const obst_avoid::vel3d::ConstPtr& msg);
			void add_velocity_to_cluster(void);
			
			void convet_image_to_pcl(cv::Mat& image);
		//---GROUND ESTIMATE
			void ground_estimation_from_pc(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d);
			void ground_estimation_from_image(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d);
		//---DEM
			void set_DEM_map(cv::Mat& image);	
			void clustering_DEM_elements(void);
			void clustering_schema(void);
			void clustering_slice(void);
			void tracking_cluster(const std::vector<optical_flow_data>& opf_data,const float v,const float omg,const double& dt);
			void convert_dem_to_pcl(void);
			void publish_slice_cluster(void);
			void clear_dem_element(void);
};
