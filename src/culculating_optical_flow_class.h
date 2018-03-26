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
	int clp_max_points;//=max_points/(cnh*cnw);
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

		
public:
	culculate_optical_flow()
		:point_size(max_points*2),clp_max_points(max_points/(cnh*cnw))
		,clp_point_size((int)(clp_max_points*10)),threshold_fp((int)(max_points*0.8))
		,th_clpimg((int)(clp_max_points*0.8))
	{
		pts.reserve(point_size);   //特徴点
		npts.reserve(point_size);  //移動後の特徴点
		sts.reserve(point_size);
		ers.reserve(point_size);
		keypoints.reserve(point_size);
		pre_z.reserve(point_size);
		cur_z.reserve(point_size);
		points.reserve(point_size);
		newpoints.reserve(point_size);
		z.reserve(point_size);
		nz.reserve(point_size);
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				cp[i][j].reserve(clp_point_size);
				cpt[i][j].reserve(clp_point_size);
				cnpt[i][j].reserve(clp_point_size);
				cz[i][j].reserve(clp_point_size);
				cpt_num[i][j].reserve(clp_point_size);
			}
		}
		jnpts.reserve(point_size);
		jnewpoints.reserve(point_size);
		tracking_count_p.reserve(point_size);
		tracking_count.reserve(point_size);
	
	}
	virtual ~culculate_optical_flow(){
	
	}
	void set_gray_images(const cv::Mat& pre_img,const cv::Mat& cur_img){//step1
		cv::cvtColor(pre_img,pre_gray_image,CV_BGR2GRAY);
		cv::cvtColor(cur_img,cur_gray_image,CV_BGR2GRAY);
	
	}
	void set_clip_images(void){//step2
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				clp_img[i][j]=pre_gray_image(cv::Rect((int)(j*(width)/cnw),(int)(i*(height)/cnh),(int)(width/cnw),(int)(height/cnh)));
			}
		}
	}
	bool culculating_observed_opticalflow(const cv::Mat& filted_pre_image,const int& ksize){
		//return true:pts.size()>0,false:pts.size()==0

		auto detector = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
		cv::Point2i ppts;
		float ppre_z;
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
			detector.detect(clp_img[i][j], keypoints);	
				for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin()
		 			itk != keypoints.end(); ++itk){
				 	ppts.x=(j*width/cnw+itk->pt.x)/ksize;
					ppts.y=i*height/cnh+itk->pt.y/ksize;
					ppre_z=filted_pre_image.at<float>(
						ppts.y,
						ppts.x
						);
					if(ppre_z>=0.5){
						pts.push_back(ppts);
						pre_z.push_back(ppre_z);
					}//end if
				}//end for
			}//end for
		}//end for

		if(!pts.size()){
			return false;
		}

		cv::calcOpticalFlowPyrLK(PreLgray,Lgray, pts, npts, sts, ers, cv::Size(13,13), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);
		
		return true;
	}
	bool culculating_moving_objects_opticalflow(const cv::Mat& filted_cur_image,const int& ksize){
		
		for(int i=0;i<pts.size();i++){
			if(sts[i]){
				
				
			}
		}
	}

	void define_variable(void);
	void image_callback(const sensor_msgs::ImageConstPtr& msg);
	void filtering_depth_image(void);
	void publish_filted_depth_image(void);
};



