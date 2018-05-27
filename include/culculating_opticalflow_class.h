#include"ros/ros.h"
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>//オプティカルフロー用
#include<opencv2/features2d/features2d.hpp>
#include <std_msgs/Empty.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>

#include"obst_avoid/point3d.h"
#include"obst_avoid/vel3d.h"
#include"obst_avoid/img_point.h"

#include"obst_avoid/points.h"
#include"obst_avoid/matching.h"

class culculate_optical_flow
{
  private:
	  ros::NodeHandle nh,nh_sub,nh_pub1;
  	ros::Publisher pub_vel;
  	ros::Subscriber sub;
		image_transport::ImageTransport it_pub1;	
		image_transport::Publisher pub1;
		ros::CallbackQueue queue_empty;
//--cluster matching
		ros::NodeHandle nh_match;
		ros::Publisher pub_match;
		
//  	cv::Mat pre_image;
//  	cv::Mat cur_image;
  	cv::Mat pre_gray_image;
  	cv::Mat cur_gray_image;

		const int width=672;
		const int height=376;
		const float f=350.505;
	//--特徴点抽出
		const int max_points=1200;//720;//800;//500
		int point_size;
		static const int cnh=9;//10;//cnh=cnw=16 -> fp.size:987
		static const int cnw=14;//18;
		const int window_size=13;
		// int clp_max_points;//=max_points/(cnh*cnw);
		//int clp_point_size;//=(int)(clp_max_points*10);
		//int threshold_fp;//=(int)(max_points*0.8);
		//const int th_clpimg;//=(int)(clp_max_points*0.8);
		std::vector<cv::Point2i> cp[cnh][cnw];
		cv::Mat clp_img[cnh][cnw];

		std::vector<cv::KeyPoint> keypoints;
		std::vector<cv::Point2f> pts;   //特徴点
		std::vector<cv::Point2f> npts;  //移動後の特徴点
		std::vector<uchar> sts;
		std::vector<float> ers;
		std::vector<float> pre_z;
		std::vector<float> cur_z;

		std::vector<cv::Point2f> points;    //特徴点
		std::vector<cv::Point2f> newpoints; //移動後の特徴点
		std::vector<float> z;//current z
		std::vector<float> nz;//new z
		std::vector<cv::Point2f> jnpts;
		std::vector<cv::Point2f> jnewpoints;
		std::vector<int> tracking_count_p;
		std::vector<int> tracking_count;
		::obst_avoid::vel3d vX;
		cv::Mat view_image;

		//add 0524
		cv::Point3f clp_vel_ave_pre[18][28];
	public:
		culculate_optical_flow();
		virtual ~culculate_optical_flow();
		void set_gray_images(const cv::Mat& pre_img,const cv::Mat& cur_img);
	//	void set_clip_images(const int& nh=cnh,const int& nw=cnw);
		void set_clip_images(void);
	//  bool obtain_feature_points(const cv::Mat& cur_depth_image,const int& nh=cnh,const int& nw=cnw);
		bool obtain_feature_points(const cv::Mat& pre_depth_image);
		void culculating_observed_opticalflow(void);
		void culculating_moving_objects_opticalflow(const cv::Mat& cur_depth_image,const double& w_v,const double& dyaw,const double& dt);

		void publish_matching_msg(const cv::Mat& cur_depth_image);
		void publish_objects_velocity(void);
		void publish_flow_image(const cv::Mat& cur_image);
		void clear_vector(void);
		cv::Mat& get_view_image(void);
		void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color);
		//add function0524
		void cul_clip_vel(const double& dt);

		//---publish syncro depth
		void publish_syncro_depth(cv::Mat& depth_image);
		//---subscribe response
		void subscribe_response(void);
		void empty_callback(const std_msgs::Empty& msg);
};
