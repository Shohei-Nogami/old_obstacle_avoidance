
#ifndef INCLUDE_IMAGE_CLASS
#define INCLUDE_IMAGE_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
//画像取得用
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>


class image_class{
	protected:
		cv::Mat cur_image;
		cv::Mat pre_image;
		ros::Time cur_time;
		ros::Time pre_time;
		ros::Duration delta_time;
	public:

		image_class();
		bool is_cur_image(void);
		bool is_pre_image(void);
		void subscribe_image(void);
		void set_cur_image(void);
		void set_pre_image(void);
		void set_image(void);
		void set_delta_time(void);
//		void set_debug_image(cv::Mat& temp_image);
		cv::Mat& get_cur_image_by_ref(void);
		cv::Mat& get_pre_image_by_ref(void);
		double get_delta_time(void);

		virtual ~image_class();
		virtual void publish_debug_image(cv::Mat& temp_image);
		virtual void define_variable(void);

	protected:
		bool PROCESS_ONCE;
		ros::NodeHandle nh_pub,nh_sub;
		image_transport::Publisher pub;
		ros::Subscriber sub;
		image_transport::ImageTransport it;
		ros::CallbackQueue queue;
		cv_bridge::CvImagePtr cvbridge_image;

		virtual void image_callback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
