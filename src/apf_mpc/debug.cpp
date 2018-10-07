#include<apf_mpc.h>
 
void APF_MPC::set_pub_mpc_debug_images(const cv::Point2i& xrit0)
{
	int W=map_wi;
	int H=map_hi;

	//mpc_debug_image = cv::Mat::zeros(cv::Size(map_hi,map_wi), CV_8UC3);
	for(int h0=0;h0<H;h0++){
		for(int w0=0;w0<W;w0++){
			//set potential
			//float pot=pot_mapt.at<float>(h0,w0);//*std::abs(sum_pot);
			float pot=pot_map.at<float>(h0,w0);//*std::abs(sum_pot);
			//std::cout<<"pot:"<<pot<<"\n";
			if(pot>0)
			{
				mpc_debug_image.at<cv::Vec3b>(h0,w0)[2] =pot*255;
			}
			else
			{
				mpc_debug_image.at<cv::Vec3b>(h0,w0)[0] =(-pot)*255;
				mpc_debug_image.at<cv::Vec3b>(h0,w0)[1] =(-pot)*255;
			}
			//set path
		}	
	}
	mpc_debug_image.at<cv::Vec3b>(xrit0.y,xrit0.x)[0] =255;
	mpc_debug_image.at<cv::Vec3b>(xrit0.y,xrit0.x)[1] =255;
	mpc_debug_image.at<cv::Vec3b>(xrit0.y,xrit0.x)[2] =255;
	
	draw_mv_obst();
	
	//ROS_INFO("publish_debug_image\n");
	publish_debug_image(mpc_debug_image);
	//ROS_INFO("published_debug_image\n");	
}
void APF_MPC::draw_mv_obst(void){
	//std::cout<<"void APF_MPC::draw_mv_obst(void){\n";
	//std::cout<<"mv_obsts.size():"<<mv_obsts.size()<<"\n";
	for(int n=0;n<mv_obsts.size();n++){
		cv::Point2i pti;
		//std::cout<<"mv_obsts[n].data.size():"<<mv_obsts[n].data.size()<<"\n";
		for(int k=0;k<mv_obsts[n].data.size();k++)
		{
			cv::Point2f pt=mv_obsts[n].data[k];
			//std::cout<<"pt0:"<<pt<<"\n";
			pt.x+=mv_obsts[n].mvx;
			pt.y+=mv_obsts[n].mvy;
			pt.x+=mv_obsts[n].mvxt;
			pt.y+=mv_obsts[n].mvyt;
			//std::cout<<"pt:"<<pt<<"\n";
			if(trans_point(pt,pti))
			{
				//std::cout<<"pti:"<<pti<<"\n";
				mpc_debug_image.at<cv::Vec3b>(pti.y,pti.x)[1] =255;
			}
		}	
	}
}
void APF_MPC::draw_mpc_path_mat(void)
{
	ros::NodeHandle n;
	ros::Rate rate(100);
	int obst_num=(int)obst_pti.size()+mv_data_size;
	clear_move_data();
	float v0=vrt;
	//float dt=(float)1/100;
	float goal_time=0;
	std::ofstream ofss("./vel_angVel.csv",std::ios::app);
	ofss<<"time"<<","<<"x"<<","<<"y"<<","<<"v"<<","<<"w"<<","<<"th_t"<<","<<std::endl;
	
	while(ros::ok()){
		//float to int
		trans_point_f_to_i(xrf,xri);
		std::cout<<"xrf,xgf:"<<xrf<<"-->"<<xgf<<"\n";
		//ゴールセルに到達したら終了
		if(xri.x==xgi.x && xri.y==xgi.y)
		{
			std::cout<<"Goal\n";
			break;
		}
		//MPC
		//ROS_INFO("get_speed...\n");
		//std::cout<<"vrt:"<<vrt<<"\n";
		v0=get_speed(xrf,vrt);
		//
		//pot_mapt=pot_map.clone();
		
		std::cout<<"v0:"<<v0<<"\n";
		/*int kk=0;
		while(ros::ok()&&kk++<100){
			set_pub_mpc_debug_images(xri);
			rate.sleep();
		}
		
		clear_move_data();
		kk=0;
		while(ros::ok()&&kk++<100){
			set_pub_mpc_debug_images(xri);
			rate.sleep();
		}
		*/
		clear_move_data();
		//pot_mapt=pot_map.clone();
		//ROS_INFO("add_mv_pot...\n");
		add_mv_pot(xri,obst_num);
		//ROS_INFO("set_grad...\n");
		set_grad(xri);
		if(check_collision(xrf))//collision
		{
			ROS_INFO("collision...\n");
			break;
		}
		
		//ロボットの命令速度算出
		float w,v;
		ROS_INFO("set_command_vel...\n");
		set_command_vel(xri,v0,v,w,th_t);
		/*
		std::cout<<"xri:"<<xri<<"\n";
		std::cout<<"pot_xri:"<<get_pot_xt(xri)<<"\n";
		std::cout<<"pot(x0,x1),(y0,y1):("<<pot_x0<<","<<pot_x1<<"),("<<pot_y0<<","<<pot_y1<<")\n";
		
		std::cout<<"grad(x,y):"<<grad_xt<<","<<grad_yt<<"\n";//<--gradに問題:解決
		*/
		std::cout<<"v0,vrt,w,th_t:"<<v0<<","<<vrt<<","<<w<<","<<th_t<<"\n";
		
		ofss<<goal_time<<","<<xrf.x+cx<<","<<xrf.y+cy<<","<<v<<","<<w<<","<<th_t<<","<<std::endl;
		
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t=th_t+w*mv_t;
		xrf.x=xrf.x + l*cos(th_t);
		xrf.y=xrf.y + l*sin(th_t);
		//速度変化
		//ロボットの速度は1時遅れ系で変化すると仮定
		//目標値v0,現在速度vrt
		float Tr=0.25/1000;//マブチ3Vモータを参考
		//vrt=vrt+(v0-vrt)*(1-exp(-Tr*dt));
		vrt=vrt+(v0-vrt)*(exp(-Tr*mv_t));
		std::cout<<"(1-exp(-Tr*dt)):"<<(1-exp(-Tr*mv_t))<<"\n";
		//debug
		//ROS_INFO("set_pub_mpc_debug_images...\n");
		past_time(mv_t);
		set_pub_mpc_debug_images(xri);
		//move obstacles
		rate.sleep();
		goal_time+=mv_t;	
	}
	std::ofstream ofs("./goal_time.csv",std::ios::app);
	ofs<<goal_time<<std::endl;
	/*
	while(ros::ok()){
		std::cout<<"goal_time:"<<goal_time<<"\n";
	}
	*/	
}
bool APF_MPC::check_collision(const cv::Point2f xrf00){

	for(int n=0;n<mv_obsts.size();n++){
		cv::Point2i pti;
		//std::cout<<"mv_obsts[n].data.size():"<<mv_obsts[n].data.size()<<"\n";
		for(int k=0;k<mv_obsts[n].data.size();k++)
		{
			cv::Point2f pt=mv_obsts[n].data[k];
			//std::cout<<"pt0:"<<pt<<"\n";
			pt.x-=cx;
			pt.y-=cy;
			pt.x+=mv_obsts[n].mvx;
			pt.y+=mv_obsts[n].mvy;
			pt.x+=mv_obsts[n].mvxt;
			pt.y+=mv_obsts[n].mvyt;
			//std::cout<<"pt:"<<pt<<"\n";
			float dis=std::sqrt((xrf00.x-pt.x)*(xrf00.x-pt.x)+(xrf00.y-pt.y)*(xrf00.y-pt.y));
			if(dis<=rr+cr)
			{
				return true;
			}
		}	
	}
	for(int k=0;k<obst_pti.size();k++)
	{
		float dis=std::sqrt((xrf00.x-obst_pti[k].x)*(xrf00.x-obst_pti[k].x)+(xrf00.y-obst_pti[k].y)*(xrf00.y-obst_pti[k].y));
		if(dis<rr+cr){
			return true;
		}
	}
	return false;
}

void APF_MPC::past_time(const float& time){
	//std::cout<<"void APF_MPC::past_time(const float& time){\n";
	clear_move_data();
	for(int n=0;n<mv_obsts.size();n++){
		/*
		float mvx=mv_obsts[n].vx*time;
		float mvy=mv_obsts[n].vy*time;
		
		for(int k=0;k<mv_obsts[n].data.size();k++)
		{
			mv_obsts[n].data[k].x+=mvx;
			mv_obsts[n].data[k].y+=mvy;
		}
		*/
		mv_obsts[n].mvxt+=mv_obsts[n].vx*time;
		mv_obsts[n].mvyt+=mv_obsts[n].vy*time;
		
	}
	//std::cout<<"void APF_MPC::past_time(const float& time){\n";
}
bool APF_MPC::set_grad0(const cv::Point2i& xti){
	
	//map size
	int W=map_wi;
	int H=map_hi;
	int delta=1;
	float delta_cell=delta*reso;
	
	cv::Point2i yti_0 =xti;
	cv::Point2i yti_1 =xti;
	//pot_xt0=pot_mapt.at<float>(xti.y,xti.x);
	std::cout<<"xti:"<<xti<<"\n";
	if(xti.y==0)
	{
		yti_1.y+=delta;
		pot_y1=pot_mapt.at<float>(xti.y+delta,xti.x);
		pot_y0=pot_mapt.at<float>(xti.y,xti.x);
		grad_yt=-get_grad_1(pot_y1,pot_y0,delta_cell);
	}
	else if(xti.y==H-delta)
	{
		yti_0.y-=delta;
		pot_y1=pot_mapt.at<float>(xti.y,xti.x);
		pot_y0=pot_mapt.at<float>(xti.y-delta,xti.x);
		grad_yt=-get_grad_1(pot_y1,pot_y0,delta_cell);
	}
	else{
		yti_1.y+=delta;
		yti_0.y-=delta;	
		pot_y1=pot_mapt.at<float>(xti.y+delta,xti.x);
		pot_y0=pot_mapt.at<float>(xti.y-delta,xti.x);
		grad_yt=-get_grad_2(pot_y1,pot_y0,delta_cell);
	}
	std::cout<<"pot(x0,x1),(y0,y1):("<<pot_x0<<","<<pot_x1<<"),("<<pot_y0<<","<<pot_y1<<")\n";
	std::cout<<"grad_xt,yt:"<<grad_xt<<","<<grad_yt<<"\n";
	
	
	cv::Point2i xti_0 =xti;
	cv::Point2i xti_1 =xti;
	if(xti.x==0)
	{
		xti_1.x+=delta;	
		pot_x1=pot_mapt.at<float>(xti.y,xti.x+delta);
		pot_x0=pot_mapt.at<float>(xti.y,xti.x);
		grad_xt=get_grad_1(pot_x1,pot_x0,delta_cell);
	}
	else if(xti.x==W-delta)
	{
		xti_0.x-=delta;
		pot_x1=pot_mapt.at<float>(xti.y,xti.x);
		pot_x0=pot_mapt.at<float>(xti.y,xti.x-delta);
		grad_xt=get_grad_1(pot_x1,pot_x0,delta_cell);
	}
	else
	{
		xti_1.x+=delta;	
		xti_0.x-=delta;
		pot_x1=pot_mapt.at<float>(xti.y,xti.x+delta);
		pot_x0=pot_mapt.at<float>(xti.y,xti.x-delta);
		grad_xt=get_grad_2(pot_x1,pot_x0,delta_cell);
	}
	if(std::isinf(grad_xt)){
		if(grad_xt>0)
			grad_xt=FLT_MAX;
		else
			grad_xt=-FLT_MAX;
	}
	if(std::isinf(grad_yt)){
		if(grad_yt>0)
			grad_yt=FLT_MAX;
		else
			grad_yt=-FLT_MAX;
	}
	if(pot_x0>max_pot||pot_x1>max_pot||pot_y0>max_pot||pot_y1>max_pot)
	{
		return true;
	}
	return false;
}
