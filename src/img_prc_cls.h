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
#include <std_msgs/Empty.h>
//自作msgファイル
#include"obst_avoid/points.h"
#include"obst_avoid/moving_points.h"
#include"obst_avoid/moving_pointsArray.h"
#include"obst_avoid/wheel_msg.h"
#include"obst_avoid/point3d.h"
#include"obst_avoid/line_point3d.h"
#include"obst_avoid/sqr_point3d.h"
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
	ros::NodeHandle nh3;
	ros::NodeHandle nh4;
	ros::NodeHandle nh5;
//subscriber
	image_transport::ImageTransport it;
	image_transport::ImageTransport it2;
	ros::Subscriber sub_Limg;//LeftImage
	ros::Subscriber sub_depth;//DepthImage
	ros::Subscriber sub_odom;
	ros::Subscriber sub_wodom;
	ros::Subscriber sub_avedepth;
//callbackqueue
	ros::CallbackQueue image_queue;
	ros::CallbackQueue depth_queue;
	ros::CallbackQueue odom_queue;
	ros::CallbackQueue wodom_queue;
	ros::CallbackQueue avedepth_queue;
//subscribe options
	ros::SubscribeOptions image_option;
	ros::SubscribeOptions depth_option;
	ros::SubscribeOptions odom_option;
	ros::SubscribeOptions wodom_option;
	ros::SubscribeOptions avedepth_option;
	ros::Time start_time;
public:
//publisher
	image_transport::Publisher pub_orgimg;
	image_transport::Publisher pub_Limg;
	image_transport::Publisher pub_Lmsk;
	image_transport::Publisher pub_dpt;
	ros::Publisher pub_odm;
	ros::Publisher pub_wheel;
	ros::Publisher pub_empty;
//variable
//画像
	cv::Mat Limg,depth_img,Limg_view;
	cv::Mat PreLimg,Predepth;//1つ前のフレームを格納
	cv::Mat Lgray,PreLgray;

	cv_bridge::CvImagePtr org_img;// Subscriber change zed topic
	cv_bridge::CvImagePtr depthimg;// Subscriber change zed topic
	cv_bridge::CvImagePtr PubLimg;
	static const int width=672;
	static const int height=376;			//image size
	bool ODOMETRY_RECEIVED;
	double prev_time;	//for image jacobian
	double new_time;	//
	double dt;			//delta time
	float f=350.505;
//	int cx=(int)354.676;
//	int cy=(int)191.479;
	double vr;
	double vl;
	double d=0.276;//車輪幅
	double w_w;
	double w_dyaw;
	double T;//LPF
	double pw;
	double w;
	double dz,dx;
	//debug
	double prev_img_time;
	double new_img_time;
	double img_dt;
//odometry
	double position_x,position_y,position_z;
	double prev_position_x,prev_position_y,prev_position_z;
	double global_dx,global_dy,global_dz;
	double roll,pitch,yaw;
	double prev_roll,prev_pitch,prev_yaw;
	double droll,dpitch,dyaw;
	double pdyaw;
	double dr;
//--特徴点抽出
	static const int max_points=1200;//720;//800;//500
	const int point_size=max_points*2;
//	static const int cn=12;
	static const int cn=12/1.2;
	static const int cnh=cn;
//	static const int cnw=cn*2;
	static const int cnw=cn*1.8;
	const int clp_max_points=max_points/(cnh*cnw);
	const int clp_point_size=(int)(clp_max_points*10);
//特徴点追加の閾値
	const int threshold_fp=(int)(max_points*0.8);
	const int th_clpimg=(int)(clp_max_points*0.8);
	std::vector<cv::Point2i> cp[cnh][cnw];
	cv::Mat clp_img[cnh][cnw];
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
	std::vector<float> z;//current z
	std::vector<float> nz;//new z
	std::vector<cv::Point2f> jnpts;
	std::vector<cv::Point2f> jnewpoints;
	std::vector<int> tracking_count_p;
	std::vector<int> tracking_count;
//--detect area exist moving objects
	std::vector<cv::Point2d> cpt[cnh][cnw];
	std::vector<cv::Point2d> cnpt[cnh][cnw];
	std::vector<double> cz[cnh][cnw];
	double p_avez[cnh][cnw];
	double p_mvarea[cnh][cnw];
	double p_pmvarea[cnh][cnw];
	double pavesize[cnh][cnw];
	double pprd_obj[cnh][cnw];
	cv::Point2d pavept[cnh][cnw];
	obst_avoid::sqr_point3d sp3d;
	std_msgs::Empty emptymsg;
	std::vector<cv::Point2i> mv_area;
	std::vector<double> opt;
	std::vector<int> cpt_num[cnh][cnw];
	std::vector<bool> is_mv_pts_p;
	std::vector<bool> is_mv_pts;
		
//--prd_prc
	int PRD_PRC_ORDER;
	static const int STC_OBST_AVOID=0;
	static const int DTCT_MV_AREA=1;
	static const int DTCT_TRUE_MV_AREA=3;
	static const int MOVING_SLOWLY=2;
	static const int PRD_MV_AREA=4;
	static const int LOCATION_BASED_TRAVEL=5;
	double idle_time;
	cv::Point2d avept[cnh][cnw];	//sum->ave
//		double avesize[cnh][cnw];		//sum->ave
	cv::Point2d dsppt[cnh][cnw];	//sum->dsp
//		double dspsize[cnh][cnw];		//sum->dsp
	double dspz[cnh][cnw];
	double avez[cnh][cnw];
//		double p_mvarea[cnh][cnw];
	int pp_mvarea[cnh][cnw];
	double ismvobj[cnh][cnw];
	double ismvline[cnw];
	double obj_pstn;
	int prd_obj[cnh][cnw];
	std::vector<int> space_begin;
	std::vector<int> space_end;
	std::vector<int> space_size;
	std::vector<cv::Point2i> area_begin;
	std::vector<cv::Point2i> area_end;
	std::vector<double> area_opt;
	std::vector<double> area_z;
	//img_plc_cls vector tracking prc_fnc
	//init value is false
	std::vector<bool> is_stc_pts;
	std::vector<bool> is_stc_pts_p;
	double min_line_z;
	double line_z[cnw];
	cv::Point2i area_begin0;//
	cv::Point2i area_end0;//
	double obj_min_z;
	int sum_cpt_num;
	double obj_size;
	double min_z;
//control wheel
	obst_avoid::wheel_msg wheelMsg;
	int vel=200;//100;//200;
	int max_vel_dif=100;
	double z_target=0.5;
	int target_vel;
	int pvel;
//	int pvel_l;
//	int pvel_r;
//image_based_travel
	cv::Point2i target_point;
	cv::Point2i ptarget_point;
	double vel0;
	double dif_v0;
	int dif_v;
//location_based_travel
	double target_sheta;
	double target_length;
	double subtarget_z;
	double subtarget_x;	
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
	void wheelodom_callback(const obst_avoid::wheel_msg::ConstPtr& msg);
	void avedepth_callback(const obst_avoid::sqr_point3d::ConstPtr& msg);
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
	//manege depth function
	void set_depth(void);
	//wheater depth exist
	bool isdepth(void);
   	//set previous image
	void setPrevdepth(void);
	//set depth cv_bridge image
	void setcvdepth(void);
	//set mat depth image
	void setmtdepth(void);
	//publish depth image
	void pub_depthimg(void);
	//Linear approximation
	void approx_depth_img(void);
	//set ave 3d
	void setave3d(void);
//-----odometry----
//set wheel odometry
	void setwodom(void);
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
//set dz dx
	void setdzdx(void);
//-----画像処理----
//画像処理
	void imageProcess();
//特徴点追跡
	void add_feature_points(void);
	void count_feature_points(void);
//移動物体予測,目標点算出
	void prd_prcess(void);
//wheel control
	void wheel_control(void);
	void image_based_travel(void);
	bool location_based_travel(void);
//publish empty msgs
	void pub_response(void);
//矢印描写用関数
	void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color);
//obstacle avoidance
	void prd_process(void);
	void culc_area_param(void);
	void culc_p_mvarea(void);
	bool synthesis_mvarea(int& i,int& j);
	bool dtct_mvarea(void);
	bool dtct_true_mvarea(void);
	void prd_mvarea(void);
	void culc_linez(void);
//	bool location_based_travel(void);
	void obj_avoid(void);
//----time----
	void gettime(void);
	void getprevtime(void);
	void culcdt(void);
	void getimgtime(void);
	void getprevimgtime(void);
	void culcimgdt(void);
//vector
	void reserve_vectors(void);
	void clear_vectors(void);
	void renew_vectors(void);
	void clear_dtctvectors(void);
//----------debug-------------
	void print_odom(void);
	//show current speed
	void show_speed(void);
	void print_dt(void);
	void print_imgdt(void);
	void print_points_size(void);
	void print_bias(void);
	void print_w(void);
	void print_clpsize(void);
	void print_cptsize(void);
	void print_sp3dsize(void);
	void write_odom(void);

};

