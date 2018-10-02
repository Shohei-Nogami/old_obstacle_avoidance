#include<apf_mpc.h>

void APF_MPC::set_mv_obstacle_data(const std::vector<cv::Point2f>& pts,const float vx0,const float vy0)
{
	mv_obst obst_temp;
	obst_temp.data=pts;
	obst_temp.vx=vx0;
	obst_temp.vy=vy0;
	obst_temp.mvx=0;
	obst_temp.mvy=0;
	obst_temp.mvxt=0;
	obst_temp.mvyt=0;
	mv_obsts[mv_obsts_size++]=obst_temp;
	mv_data_size+=(int)pts.size();
}
void APF_MPC::end_set_mv_obstacle_data(void)
{
	mv_obsts.resize(mv_obsts_size);
}
void APF_MPC::clear_mv_obstacle_data(void)
{
	mv_obsts.clear();
	mv_obsts.resize(map_wi*map_hi/reso/reso);
	mv_obsts_size=0;
	mv_data_size=0;
}
void APF_MPC::move_obstacle_data(float& time)
{
	for(int k=0;k<mv_obsts.size();k++){
		mv_obsts[k].mvx+=mv_obsts[k].vx*time;
		mv_obsts[k].mvy+=mv_obsts[k].vy*time;
	}
}
void APF_MPC::clear_move_data(void)
{
	for(int k=0;k<mv_obsts.size();k++){
		mv_obsts[k].mvx=0;
		mv_obsts[k].mvy=0;
	}
}
void APF_MPC::set_static_obstacle_data(const cv::Point2f& data){
	set_obstacle_data(data);
}
