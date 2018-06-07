#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"obst_avoid/point3d.h"
#include"obst_avoid/object_info.h"
#include"obst_avoid/objects_info.h"
#include"time_class.h"
#include <pcl_ros/point_cloud.h>
//#include<Eigen/Core>
//#include<Eigen/Dense>
#include<fstream>//file input output

class avoid{
	private:
		ros::NodeHandle nh_sub,nh_pub;
		
		ros::Publisher pub,pub_pcl;
		ros::Subscriber sub;
		ros::CallbackQueue queue;



};
