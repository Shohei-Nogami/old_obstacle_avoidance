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
#include"obst_avoid/cluster_with_vel.h"
#include"obst_avoid/point3d.h"
#include"odometry_class.h"

//#include <pcl_ros/point_cloud.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

class vfh_class{
	private:
		//pcl::PointCloud<pcl::PointXYZRGB> pcl_data;
		//pcl::PointCloud<pcl::PointXYZ> pcl_data;
		ros::NodeHandle nh_pub,nh_pub2,nh_sub,nh_pubpcl,nh_pub_w;
		ros::Subscriber sub;
		ros::Publisher pc_pub,pub_wheel;
		ros::CallbackQueue queue;
		cv::Mat grid_map;
		cv::Mat grid_map_view,binary_grid_map_view;
		image_transport::ImageTransport it_pub;
		image_transport::Publisher pub;
		image_transport::ImageTransport it_pub2;
		image_transport::Publisher pub2;
		::obst_avoid::cluster_with_vel cluster;
		cv::Mat depth_image;
		int grid_resolution;//
		float grid_size;
		float grid_cell_size;
		
		const int width=672;
		const int height=376;
		const float f=350.505;
		const int ksize=3;
		bool EXECUTED_CALLBACK;
		uint8_t/*uchar*/ binary_threshold;

//parameter of trajectory
		int vfh_resolution=90+1;
		float temp_v=0.2;
		float temp_v_dif_max=0.2;
		float d=0.138;
		int good_trajectory_num;
//		float temp_vl[vel_patern],temp_vr[vel_patern];
		std::vector<double> delta_theta;
		std::vector<int> max_process_n;
		std::vector<int> rank_trajectory;
		int max_angle=45;
		int min_angle=-45;
		
		
	  	std::vector<bool> not_select_angle;
		
//parameter of collision avoidanace 
		float R=0.2;
		float d_r=0.1;
		int jn_Rd;
		std::vector<int> in_Rd;
		float target_x;
		float target_y;
		int process_n;
//wheel msg			
		float vel;
		float ang_vel;
		float max_dif=0.2;
		float length=0.2;
		int vl,vr;
		float z_th=1;

		::obst_avoid::wheel_msg wheelMsg;
	public:
		vfh_class();
		virtual ~vfh_class();
		//void subscribe_pcl(void);
		void subscribe_cluster(void);
		void cluster_callback(const obst_avoid::cluster_with_vel::ConstPtr& msg);
		bool is_cluster(void);
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
			
		void publish_cloud(void);

		void simulate_obstacle(void);

		void publish_velocity(void);
		
		
		void publish_velocity(float& vel,float& angvel);
		void set_param(const int& min_ang,const int& max_ang,const int& reso,const float& rob_r,const float& mrg_r,const float& mv_length,const float& max_vdif);
		void clear_grid_map(void);
		void set_grid_map(const std::vector<obst_avoid::point3d>& pt);
		int select_best_trajectory(const cv::Point2f& x0, const float& theta0, const cv::Point2f& xp,const float w_target, const float w_angle);

		void set_not_select_angle(std::vector<bool>& not_select_angle_temp);
};

