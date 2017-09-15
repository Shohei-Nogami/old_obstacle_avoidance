#include"ros/ros.h"
#include<nav_msgs/Odometry.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>//オプティカルフロー用
#include<opencv2/features2d/features2d.hpp>
#include<typeinfo>//型調べ
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//#include<msg/moving_point.h>
#include"obst_avoid/points.h"
#include"obst_avoid/moving_points.h"
#include"obst_avoid/moving_pointsArray.h"
#include<fstream>//file input output

#define PI 3.141593

class ImageProcesser 
{
	ros::NodeHandle nh;			
	image_transport::ImageTransport it;
	image_transport::Subscriber sub_Limg;//LeftImage
	image_transport::Subscriber sub_depth;//DepthImage
	ros::Subscriber sub_odom;	
	image_transport::Publisher pub;
	image_transport::Publisher pub_Limg;
	image_transport::Publisher pub_Lmsk;
	ros::Time start_time;
public:
	int maxfps=0;			//to read process time 
	int minfps=100;			//
	cv::Mat PreLimg,PreRimg;//1つ前のフレームを格納
	cv_bridge::CvImagePtr org_img;// Subscriber change zed topic
	cv_bridge::CvImagePtr depthimg;// Subscriber change zed topic
	bool DEPTH_RECEIVED;
	bool ODOMETRY_RECEIVED;
	int PROCESS_ORDER;
	int width;			//image size
	int height;			//
	double prev_time;	//for image jacobian
	double new_time;	//
	double dt;			//delta time
	int f=(int)350.505;
	int cx=(int)354.676;
	int cy=(int)191.479;
	double vl=0.0;		//velocity left
	double vr=0.0;		//			right
	double position_x;	
	double position_y;	
	double position_z;	
	double prev_position_x;	
	double prev_position_y;	
	double prev_position_z;
	double visual_dx;
	double visual_dy;
	double visual_dz;
	double roll;
	double pitch;
	double yaw;
	double prev_roll;
	double prev_pitch;
	double prev_yaw;
	double droll;
	double dpitch;
	double dyaw;
//debug
	float max_value_x,min_value_x,max_value_y,min_value_y;
	double current_z;
//
	::obst_avoid::moving_pointsArray movepointsArray;
	::obst_avoid::moving_points movepoints;
	::obst_avoid::points point;
//
	std::ofstream ofs;//("output_data.csv");

	ImageProcesser();//コンストラクタ
	~ImageProcesser()//デストラクタ
	{
	}
//矢印描写用関数
	void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color);

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void imageProcess();
//depth画像をprivate変数に格納
	void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
	void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

};
