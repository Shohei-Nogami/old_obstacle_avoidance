#include"ros/ros.h"
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
#define	value_lpf (3)
 
class ImageProcesser
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub_Limg;//LeftImage
	image_transport::Subscriber sub_depth;//DepthImage
	ros::Subscriber sub_odom;
	ros::Time start_time;
public:
//publisher
	image_transport::Publisher pub_orgimg;
	image_transport::Publisher pub_Limg;
	image_transport::Publisher pub_Lmsk;
	image_transport::Publisher pub_dpt;
	ros::Publisher pub_odm;
//cliant
	ros::ServiceClient imgclient;
	ros::ServiceClient dptclient;
	ros::ServiceClient odmclient;
//variable
	int maxfps=0;			//to read process time
	int minfps=100;			//
//	const int value_lpf=3;//LPF
	cv::Mat Limg,depth_img;
	cv::Mat PreLimg,PreRimg;//1つ前のフレームを格納
	cv_bridge::CvImagePtr org_img;// Subscriber change zed topic
	cv_bridge::CvImagePtr depthimg;// Subscriber change zed topic
	cv_bridge::CvImagePtr lpf_img[value_lpf];//
	bool DEPTH_RECEIVED;
	bool ODOMETRY_RECEIVED;
	int PROCESS_ORDER;
	int width,height;			//image size
	double prev_time;	//for image jacobian
	double new_time;	//
	double dt;			//delta time
	int f=(int)350.505;
	int cx=(int)354.676;
	int cy=(int)191.479;
	double vl=0.0;		//velocity left
	double vr=0.0;		//			right
	double position_x,position_y,position_z;
	double prev_position_x,prev_position_y,prev_position_z;
	double global_dx,global_dy,global_dz;
	double roll,pitch,yaw;
	double prev_roll,prev_pitch,prev_yaw;
	double droll,dpitch,dyaw;
	int img_srv_count=0;
//debug
	float max_value_x,min_value_x,max_value_y,min_value_y;
	double current_z;
//送信用msg
	::obst_avoid::moving_pointsArray movepointsArray;
	::obst_avoid::moving_points movepoints;
	::obst_avoid::points point;
//service定義
	obst_avoid::image imgsrv;
	obst_avoid::image dptsrv;
	obst_avoid::odometry odmsrv;
//ファイル出力
	std::ofstream ofs;

	ImageProcesser();//コンストラクタ
	~ImageProcesser()//デストラクタ
	{
	}
//image,depth,odometry取り込み用メソッド
//---image----
//set original image	要改善(1つ前の画像の有無等)
	void setimage(void){
		if(isLimage())
			setPrevimage();
		
		set_orgimg();
		set_Limg();
	//set LPF image
	}
	//set Left image
	void set_Limg(void){
		Limg=org_img->image.clone();
	}
	//set original image
	void set_orgimg(void){
		org_img=cv_bridge::toCvCopy(imgsrv.response.imgmsg,sensor_msgs::image_encodings::BGR8);
	}
//wheater image exist
	bool isPrevimage(void){
		if(PreLimg.empty())
			return false;
		else
			return true;
	}
	bool isLimage(void){
		if(Limg.empty())
			return false;
		else
			return true;
	}
//set previous image
	void setPrevimage(void){
		PreLimg=org_img->image.clone();
	}
	void pub_org_img(void){
		pub_orgimg.publish(org_img->toImageMsg());
	}
//----depth----
//set depth image
	void setdepth(void){
		depthimg=cv_bridge::toCvCopy(dptsrv.response.imgmsg,sensor_msgs::image_encodings::TYPE_32FC1);
	}
	//set cv::Mat depth_img
	void setdepth_img(void){
	  depth_img=depthimg->image.clone();
	}
	void pub_depthimg(void){
		pub_dpt.publish(depthimg->toImageMsg());
	}
//-----odometry----
//set previous odometry
	void setPrevodom(void){
		prev_position_x=position_x;
		prev_position_y=position_y;
		prev_position_z=position_z;
		prev_roll=roll;
		prev_pitch=pitch;
		prev_yaw=yaw;
	}
//set odometry
	void setodom(void){
		if(isOdom()){
			//---get previous time
			getprevtime();
			//get time
			gettime();
			//culculation dt
			culcdt();
			//---一つ前のodometryを格納
			setPrevodom();
			//現在のodometryを格納
			setparam();
			//set delta r,p,y
			setdpose();
			//set global odometry
			setglobalodom();
			bool front=pose_detection(position_x,position_y,prev_yaw);
			if(!front)
				dxsignchange();
		}
		else{
			//現在のodometryを格納
			setparam();
			setodomrcvd();
			//get time
			gettime();
		}
	}
//set x,y,z,r,p,y
	void setparam(void){
		//現在のodometryを取得
		double r,p,y;//一時的に格納するための変数
		tf::Quaternion quat(
			odmsrv.response.odmmsg.pose.pose.orientation.x,
			odmsrv.response.odmmsg.pose.pose.orientation.y,
			odmsrv.response.odmmsg.pose.pose.orientation.z,
			odmsrv.response.odmmsg.pose.pose.orientation.w);
		tf::Matrix3x3(quat).getRPY(r,p,y);
		//set
		position_x=odmsrv.response.odmmsg.pose.pose.position.x;
		position_y=odmsrv.response.odmmsg.pose.pose.position.y;
		position_z=odmsrv.response.odmmsg.pose.pose.position.z;
		roll=r;
		pitch=p;
		yaw=y;
	}
//set delta r,p,y
	void setdpose(void){
		droll=roll-prev_roll;
		dpitch=pitch-prev_pitch;
		dyaw=yaw-prev_yaw;
	}
//set global dx,dy
	void setglobalodom(void){
		double r=sqrt((position_x*position_x
			-2*position_x*prev_position_x
			+prev_position_x*prev_position_x)
			+(position_y*position_y
			-2*position_y*prev_position_y
			+prev_position_y*prev_position_y));
		global_dx=-r*cos(-dyaw);
		global_dy=r*sin(-dyaw);
	}
//wheater image exist
	bool isOdom(void){
		if(ODOMETRY_RECEIVED)
			return true;
		else
			return false;
	}
//set odometryflag
	void setodomrcvd(void){
		ODOMETRY_RECEIVED=true;
	}
//odometry dx's sign change
	void dxsignchange(void){
		global_dx=(-global_dx);
	}
//進行の向きを取得
	bool pose_detection(double position_x,double position_y,double prev_yaw)
	{
		bool status=true;
		if(-prev_yaw<(PI/2.0)&&-prev_yaw>0&&
			(-(position_y-prev_position_y)>0||(position_x-prev_position_x)>0)){
			if((-prev_yaw)-atan(
				(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				status=false;
		}
		if(-prev_yaw>-(PI/2.0)&&-prev_yaw<0&&
			(-(position_y-prev_position_y)<0||(position_x-prev_position_x)>0)){
			if(-(-prev_yaw)+atan(
				(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				status=false;
		}
		if(-prev_yaw>-(PI)&&-prev_yaw<-(PI/2.0)&&
			(-(position_y-prev_position_y)<0||(position_x-prev_position_x)<0)){
			if(PI+(-prev_yaw)-atan(
				(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				status=false;
		}
		if(-prev_yaw<(PI)&&-prev_yaw>(PI/2.0)&&
		(-(position_y-prev_position_y)>0||(position_x-prev_position_x)<0)){
			if(PI-(-prev_yaw)+atan(
				(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				status=false;
		}
		if(-prev_yaw==0&&(position_x-prev_position_x)>0)
			status=false;

		if(-prev_yaw==PI/2.0&&-(position_y-prev_position_y)>0)
			status=false;

		if(-prev_yaw==-PI/2.0&&-(position_y-prev_position_y)<0)
			status=false;

		if((-prev_yaw==-PI||-prev_yaw==PI)&&(position_x-prev_position_x)<0)
			status=false;

	}
	void print_odom(void){
		ROS_INFO("(x,y,z,|,r,p,y):(%f,%f,%f,|,%f,%f,%f)",
			position_x,position_y,position_z,roll,pitch,yaw);
	}
	//show current speed
	void show_speed(void){
		ROS_INFO("(v,w:(%f,%f),(dz,dw,dt):(%f,%f,%f)",global_dx/dt,dyaw/dt,global_dz,dyaw,dt);
	}
//-----画像処理----
//画像処理
	void imageProcess();
//矢印描写用関数
	void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color){
    int thickness=4;
    int lineType=8;
    int shift=0;
    cv::line(*img,pt1,pt2,color,thickness,lineType,shift);
    float vx = (float)(pt2.x - pt1.x);
    float vy = (float)(pt2.y - pt1.y);
    float v = sqrt( vx*vx + vy*vy );
    float ux = vx / v;
    float uy = vy / v;
		//矢印の幅の部分
    float w=5,h=10;
    cv::Point2i ptl,ptr;
    ptl.x = (int)((float)pt2.x - uy*w - ux*h);
    ptl.y = (int)((float)pt2.y + ux*w - uy*h);
    ptr.x = (int)((float)pt2.x + uy*w - ux*h);
    ptr.y = (int)((float)pt2.y - ux*w - uy*h);
		//矢印の先端を描画する
		//--例外処理(!(v==0))
		if(!(v==0)){
	    	cv::line(*img,pt2,ptl,color,thickness,lineType,shift);
	    	cv::line(*img,pt2,ptr,color,thickness,lineType,shift);
		}
	}
//count image service
	bool iscount(void){
		if(img_srv_count<value_lpf)
			return false;
		else
			return true;
	}
	void reset_img_srv_count(void){
		img_srv_count=0;
}
//----service----
//call imageservice
	bool callimgsrv(void){
	        imgsrv.request.cnt=img_srv_count;
		if(!imgclient.call(imgsrv)){
			ROS_INFO("error: falid image received");
			return false;
		}
		return true;
}
//call depthimageservice
	bool calldptsrv(void){
		if(!dptclient.call(dptsrv)){
			ROS_INFO("error: falid depth received");
			return false;
		}
		else
			return true;
	}
//call odometryservice
	bool callodmsrv(void){
		if(!odmclient.call(odmsrv)){
			ROS_INFO("error: falid odometry received");
			return false;
		}
		else
			return true;
	}
//----time----
	void gettime(void){
		ros::Duration time = ros::Time::now()-start_time;
		new_time=time.toSec();
	}
	void getprevtime(void){
		prev_time=new_time;
	}
	void culcdt(void){
		dt=new_time-prev_time;
	}
};
