#ifndef INCLUDE_APF_CLASS
#define INCLUDE_APF_CLASS

#include <stdio.h>
#include<iostream>
#include<cmath>
#include<fstream>//file input utput

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>

#include<ros/ros.h>
#include<omp.h>
class APF{
	protected:
		//apf.cpp
		float map_wf,map_hf;//map size float
		float reso;//resolution
		int map_wi,map_hi;//map size int
		float cr;//cell radius
		float max_pot;
		float min_pot;
		//grid_map.cpp
		cv::Mat grid_map;//グリッドマップ
		//potential_map.cpp
		cv::Mat pot_map;//コストマップ
		//gradient_map.cpp
		cv::Mat grad_map[2];//勾配マップ
		
		//debug
		cv::Mat debug_image;
		cv_bridge::CvImagePtr cvbridge_image;
		ros::NodeHandle nh_pub;
		ros::Publisher pub;
		//set_param.cpp
		float cx,cy;//center point x,y
		cv::Point2f xgf;//ゴール座標 float
		cv::Point2i xgi;//ゴール座標 int
		cv::Point2f xr;//ロボットの位置 float global coordinate
		cv::Point2f xrf;//ロボットの位置 float gridmap coordinate
		cv::Point2i xri;//ロボットの位置 int
		float rr;//robot radiu
		float vrt;//robot speed
		float th_t;//robot angular
		float max_w;//robot angular speed (max)
		float mv_t;//movement time(dt)
		
		float sum_pot;//sum_potential
		std::vector<cv::Point2i> obst_pti;
	public:
	
		//apf.cpp
		APF();
		APF(float width,float height,float resolution);
		~APF();
		//set_param.cpp
		void set_grid_param(float width,float height,float resolution);
		void set_center_point(float cpx,float cpy);
		void set_goal(cv::Point2f& goal_2f);
		bool set_robot_param(float x,float y, float r,float vt0,float th_t0);
		void set_command_limit(float max_dif_vel);
		void set_mov_time(float time);
		//coordinate_transform.cpp
		bool trans_point(const cv::Point2f& pt,cv::Point2i& pti);
		bool trans_point(const cv::Point2f& pt,cv::Point2i& pti,cv::Point2f& ptf);
		void trans_point_f_to_i(const cv::Point2f& ptf,cv::Point2i& pti);
		//grid_map.cpp
		bool check_gridmap_format(cv::Mat& map);
		void clear_grid_map(void);
		void set_grid_map(cv::Mat& map);
		void set_obstacle_data(const cv::Point2f& data);
		//potential_map.cpp
		float culc_dis(const int& w0,const int& h0,const int& w1,const int& h1);
		float culc_fr(const float& dis,const int& obstNum);
		float culc_fr(const int& w0,const int& h0,const int& w1,const int& h1,const int& obstNum);
		float culc_L1dis(const int& w0,const int& h0,const int& w1,const int& h1);
		float culc_L1fr(float& dis,const int& obstNum);
		float culc_chvdis(const int& w0,const int& h0,const int& w1,const int& h1);
		float culc_chvfr(const int& w0,const int& h0,const int& w1,const int& h1,const int& obstNum);
		float culc_fa(float dis);
		float culc_fa(const int& w0,const int& h0,const int& w1,const int& h1);
		void search_obst_pt(void);
		void create_pot_map(void);
		//gradient_map.cpp
		float get_grad_1(float& x0,float& x1,float& delta);
		float get_grad_2(float& x0,float& x1,float& delta);
		void create_grad_map(void);
		//path_planning.cpp
		virtual void set_command_vel(cv::Point2i& xri0,float& v,float& w);
		virtual void draw_path_mat(void);
		
		//debug
		void set_pub_debug_images(void);
		void publish_debug_image(const cv::Mat& temp_image);
};

#endif

