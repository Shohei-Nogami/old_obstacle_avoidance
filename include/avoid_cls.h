#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"obst_avoid/point3d.h"
#include"obst_avoid/object_info.h"
#include"obst_avoid/objects_info.h"
#include"obst_avoid/filted_objects_info.h"
#include"obst_avoid/filted_object_info.h"
#include"time_class.h"
#include <pcl_ros/point_cloud.h>
//#include<Eigen/Core>
//#include<Eigen/Dense>
#include<fstream>//file input output
#include"vfh+_class.h"

class avoid {
private:
	ros::NodeHandle nh_sub, nh_pub;

	ros::Publisher pub, pub_pcl;
	ros::Subscriber sub;
	ros::CallbackQueue queue;

	obst_avoid::filted_objects_info obj_info;

	std::vector<int> obstacle_status;
	vfh_class vfh;

	std::vector<float> obj_eq_a;//ŒX‚«

	float selected_angle_i;

public:
	avoid();
	~avoid();
	void subscribe_objects(void);
	void objects_callback(const obst_avoid::filted_objects_info::ConstPtr& msg);
	bool dicriminate_obstacle(void);
	void culculate_equation(void);
	float select_route(void);
	void culculate_intersection(void);
};
