
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
	
	std::vector<float> temp_vel;//vel list 
  	std::vector<bool> not_select_angle;
	
	
	float selected_vel;
	
	int ROBOT_STATUS;
	
	//dicriminate_obstacle
	std::vector<int> obstacle_status;
	vfh_class vfh;
	//vfh parameter
	int min_angle=-45;
	int max_angle=45;
	int vfh_resolution=90;
	float w_target = 0.7;
	float w_angle = 0.5;
	float robot_r=0.2;
	float d_r=0.1;
	float mv_length=0.2;
	float max_vel_dif=0.2;

	//set_equation
	std::vector<float> obj_eq_a;//ŒX‚«
	//select_route
	int selected_angle_i;
	float selected_angle;
	//set_intersection
	std::vector<cv::Point2f> x_c;
	//set_intersection_time
	std::vector<double> t_c;
	//set_dengerous_time_range
	std::vector<double> t_c_min;
	std::vector<double> t_c_max;
	//check_collision
	std::vector<std::vector<float>> t_c_min_angle;//[angle][vel_num]
	//select_safety_vel
  	int selected_vel_num;
  	//set_safety_vel
	float safety_vel;
	float safety_angle;
  	
public:
	avoid();
	~avoid();
	void set_param_vfh(void);
	void subscribe_objects(void);
	void objects_callback(const obst_avoid::filted_objects_info::ConstPtr& msg);
	bool dicriminate_obstacle(void);
	void set_equation(void);
	void set_grid_map(void);

	void set_not_select_angle_to_vfh(void);

	void select_route(void);
	void set_intersection(void);
	void set_intersection_time(void);
	void labeling_dangerous_obstacle(void);
	void set_dengerous_time_range(void);
	bool check_collision(void);
	bool is_collision(cv::Point2f& xr,cv::Point2f& xo,double& objs_r);
	
	
	bool change_selected_vel(void);
	void set_selected_vel(void);
	void set_default_vel_num(void);
	
	bool select_safety_vel(void);
	bool set_dangerous_angle(void);
	void clear_not_select_angle(void);
	void set_safety_vel(void);
	void set_stop_vel(void);

	void publish_velocity(void);
	void init_data(void);
	
	void publish_grid_map(void);
};

