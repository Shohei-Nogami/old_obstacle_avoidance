#include<apf.h>

void APF::set_grid_param(float width,float height,float resolution){
	//マップデータの設定と初期化
	map_wf=width;
	map_hf=height;
	reso=resolution;
	map_wi=(int)(map_wf/resolution);
	map_hi=(int)(map_hf/resolution);
	cr=std::sqrt(2)*reso/2;
	
	
	//cv::Mat
	//int
	cv::Mat m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC1);
	grid_map=m_temp.clone();
	
	//float
	m_temp = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_32FC1);
	pot_map=m_temp.clone();
	grad_map[0]=m_temp.clone();
	grad_map[1]=m_temp.clone();
	//debug
	debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
}
//グリッドマップの中心座標（クローバル座標）
void APF::set_center_point(float cpx,float cpy){
	cx=cpx;
	cy=cpy;
}

void APF::set_goal(cv::Point2f& goal_2f)
{
	if(trans_point(goal_2f,xgi,xgf))
	{
		
	}
	else
	{
		std::cout<<"GoalPoint is not in grid map\n";
	}
}
bool APF::set_robot_param(float x,float y, float r,float vt0,float th_t0)
{
	xr.x=x;
	xr.y=y;
	rr=r;
	vrt=vt0;
	th_t=th_t0;//反時計回り(0~360)
	
	if(trans_point(xr,xri,xrf))
	{
		//std::cout<<"xr,xri,xrf:"<<xr<<","<<xri<<","<<xrf<<"\n";
		return true;
	}
	else{
		std::cout<<"RobotPoint is not in grid map\n";
		return false;
	}	
}
void APF::set_command_limit(float dif_vel)
{
	float d=0.138;
	max_w=dif_vel/(2*d);
}
void APF::set_mov_time(float time)
{
	mv_t=time;
}
