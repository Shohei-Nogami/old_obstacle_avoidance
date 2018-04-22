#include"ros/ros.h"
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>//オプティカルフロー用
#include<opencv2/features2d/features2d.hpp>


class culculate_optical_flow
{
  private:
//  	cv::Mat pre_image;
//  	cv::Mat cur_image;
  	cv::Mat pre_gray_image;
  	cv::Mat cur_gray_image;

	const int width=672;
	const int height=376;
//--特徴点抽出
	const int max_points=1200;//720;//800;//500
	int point_size;
	const int cnh=10;
	const int cnw=18;
	// int clp_max_points;//=max_points/(cnh*cnw);
	int clp_point_size;//=(int)(clp_max_points*10);
	int threshold_fp;//=(int)(max_points*0.8);
	const int th_clpimg;//=(int)(clp_max_points*0.8);
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
  std::vector<cv::Point3f> dX;
	cv::Mat view_image;

public:
	culculate_optical_flow();
	virtual ~culculate_optical_flow();
	void set_gray_images(const cv::Mat& pre_img,const cv::Mat& cur_img);
	void set_clip_images(const int& nh=cnh,const int& nw=cnw);
  bool obtain_feature_points(const int& nh=cnh,const int& nw=cnw);
	void culculating_observed_opticalflow(const int& window_size=13);
  void culculating_moving_objects_opticalflow(const cv::Mat& cur_image);

	void publish_flow_image(const cv::Mat& cur_image);
  void clear_vector(void);
	cv::Mat& get_view_image(void);
  void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color);

};
