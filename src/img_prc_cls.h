#include"ros/ros.h"
#include <ros/callback_queue.h>
//画像取得用
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>//オプティカルフロー用
#include<opencv2/features2d/features2d.hpp>
#include<typeinfo>//型調べ
//odometry取得用
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//ros msgファイル
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/image_encodings.h>
//自作msgファイル
#include"obst_avoid/points.h"
#include"obst_avoid/moving_points.h"
#include"obst_avoid/moving_pointsArray.h"
//service
#include<obst_avoid/image.h>
#include<obst_avoid/odometry.h>
//ファイル出力用
#include<fstream>//file input output
//円周率
#define PI 3.141593
 
class ImageProcesser
{
	ros::NodeHandle nh;
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
//subscriber
	image_transport::ImageTransport it;
	ros::Subscriber sub_Limg;//LeftImage
	ros::Subscriber sub_depth;//DepthImage
	ros::Subscriber sub_odom;
//callbackqueue
	ros::CallbackQueue image_queue;
	ros::CallbackQueue depth_queue;
	ros::CallbackQueue odom_queue;
//subscribe options
	ros::SubscribeOptions image_option;
	ros::SubscribeOptions depth_option;
	ros::SubscribeOptions odom_option;

	ros::Time start_time;
public:
//publisher
	image_transport::Publisher pub_orgimg;
	image_transport::Publisher pub_Limg;
	image_transport::Publisher pub_Lmsk;
	image_transport::Publisher pub_dpt;
	ros::Publisher pub_odm;
//variable
//画像
	cv::Mat Limg,depth_img,Limg_view;
	cv::Mat PreLimg,PreRimg;//1つ前のフレームを格納
	cv_bridge::CvImagePtr org_img;// Subscriber change zed topic
	cv_bridge::CvImagePtr depthimg;// Subscriber change zed topic
	cv_bridge::CvImagePtr PubLimg;
	int width,height;			//image size
	bool ODOMETRY_RECEIVED;
	double prev_time;	//for image jacobian
	double new_time;	//
	double dt;			//delta time
//	int f=(int)350.505;
	int f=699.209;
	int cx=(int)354.676;
	int cy=(int)191.479;
	double vl=0.0;		//velocity left
	double vr=0.0;		//right
//odometry
	double position_x,position_y,position_z;
	double prev_position_x,prev_position_y,prev_position_z;
	double global_dx,global_dy,global_dz;
	double roll,pitch,yaw;
	double prev_roll,prev_pitch,prev_yaw;
	double droll,dpitch,dyaw;
	int img_srv_count=0;
//vector point,z
	std::vector<cv::Point2f> pts;   //特徴点
	std::vector<cv::Point2f> npts;  //移動後の特徴点
	std::vector<uchar> sts;
	std::vector<float> ers;
	std::vector<cv::KeyPoint> keypoints;
//Provisional z
	std::vector<float> pz;

    std::vector<cv::Point2f> points;    //特徴点
    std::vector<cv::Point2f> newpoints; //移動後の特徴点
	std::vector<float> z;

//debug
	float max_value_x,min_value_x,max_value_y,min_value_y;
	double current_z;
//ファイル出力
	std::ofstream ofs;

	ImageProcesser();//コンストラクタ

	~ImageProcesser()//デストラクタ
	{
	}
//callback function
	void image_callback(const sensor_msgs::ImageConstPtr& msg);
	void depth_callback(const sensor_msgs::ImageConstPtr& msg);

	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
//image,depth,odometry取り込み用メソッド
//---image----
//set original image	要改善(1つ前の画像の有無等)
	void setimage(void);
	//set original image
	void set_orgimg(void);
	//set previous image
	void setPrevimage(void);
	//set Left image
	void set_Limg(void);
	//wheater image exist
	bool isPrevimage(void);
	bool isLimage(void);
	//publish original image
	void pub_org_img(void);
	//publish view left image
	void pub_left_img(void);

//----depth----
//set depth image
	void setdepth(void);
	//set mat depth image
	void setdepth_img(void);
	//publish depth image
	void pub_depthimg(void);
//-----odometry----
//set previous odometry
	void setPrevodom(void);
//set odometry
	void setodom(void);
//set x,y,z,r,p,y
	void setparam(void);
//set delta r,p,y
	void setdpose(void);
//set global dx,dy
	void setglobalodom(void);
//wheater image exist
	bool isOdom(void);
//set odometryflag
	void setodomrcvd(void);
//odometry dx's sign change
	void dxsignchange(void);
//進行の向きを取得
	bool pose_detection(double position_x,double position_y,double prev_yaw);
//----------debug-------------
	void print_odom(void);
	//show current speed
	void show_speed(void);
	void print_dt(void);
//-----画像処理----
//画像処理
	void imageProcess();
//矢印描写用関数
	void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color);
//----time----
	void gettime(void);
	void getprevtime(void);
	void culcdt(void);
//memory
	void reserve_vectors(void);
	void clear_vectors(void);

};
