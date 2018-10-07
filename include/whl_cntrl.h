#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"obst_avoid/cluster_with_vel.h"

#include"obst_avoid/wheel_msg.h"
//#include <pcl_ros/point_cloud.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

class wheel_control_class{

	private:
		ros::NodeHandle nh_sub;
		ros::NodeHandle nh_pub;
		ros::Subscriber sub_vel;
		ros::CallbackQueue queue;
		ros::Publisher pub_wheel;
		::obst_avoid::wheel_msg wheelMsg;
		float max_dif=0.2;

	public:
		wheel_control_class();
		~wheel_control_class();

		void subscribe_wheel_velocity(void);
		void velocity_callback(const obst_avoid::wheel_msg::ConstPtr& msg);
		bool set_msg(void);
		void publish_velocity(void);
		
};




