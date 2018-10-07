#include<apf_mpc.h>

float& APF_MPC::get_pot_xt(const cv::Point2i& xti){
	return pot_mapt.at<float>(xti.y,xti.x);
}
bool APF_MPC::set_grad(const cv::Point2i& xti){
	
	//map size
	int W=map_wi;
	int H=map_hi;
	int delta=1;
	float delta_cell=delta*reso;
	
	cv::Point2i yti_0 =xti;
	cv::Point2i yti_1 =xti;
	//pot_xt0=pot_mapt.at<float>(xti.y,xti.x);
	
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
	double th_pot=max_pot*((int)obst_pti.size()+mv_data_size);
	if(pot_x0>th_pot||pot_x1>th_pot||pot_y0>th_pot||pot_y1>th_pot)
	{
		return true;
	}
	return false;
}

//retunr sum cost
double APF_MPC::culc_cost(cv::Point2f& xrft0,const float v0,const float& time_range)
{
	ros::NodeHandle n;
	ros::Rate rate(1000);
	double sum_cost=0;
	float tr=time_range;
	cv::Point2f xrft=xrft0;
	cv::Point2i xrit=xri;
	float th_t0=th_t;
	//init mv data
	clear_move_data();
	int obst_num=(int)obst_pti.size()+mv_data_size;
	//ROS_INFO("culc_cost...while\n");
	//std::cout<<"xrft0,xrft:"<<xrft0<<","<<xrft<<"\n";
	//std::cout<<"xrit,xri:"<<xrft0<<","<<xrft<<"\n";
	float vrate=2;
	while(ros::ok()&&tr>0)
	{
		//float to int
		trans_point_f_to_i(xrft,xrit);
		//std::cout<<"xrft,xgf:"<<xrft<<"-->"<<xgf<<"\n";
		//std::cout<<"xrit:"<<xrit<<"\n";
		//std::cout<<"i:"<<i++<<"\n";
		//set pot map(t)
		pot_mapt=pot_map.clone();
		//ROS_INFO("add_mv_pot...while\n");
		add_mv_pot(xrit,obst_num);
		//add cost
		//ROS_INFO("sum_cost...while\n");
		//std::cout<<"sum_cost:"<<sum_cost<<"\n";
		sum_cost+=get_pot_xt(xrit);
		//ゴールセルに到達したら終了
		if(xrit.x==xgi.x && xrit.y==xgi.y)
		{
			std::cout<<"Goal\n";
			if(sum_cost>0){
				sum_cost/=vrate;
			}
			else{
				sum_cost*=vrate;
			}
			return -DBL_MAX*(tr/time_range);
		}
		//ROS_INFO("set_grad...while\n");
		if(set_grad(xrit))//collision
		{
			ROS_INFO("collision...MPC\n");
			return DBL_MAX-(time_range-tr);
		}
		//std::cout<<"FLT_MAX,isninf x,y:"<<FLT_MAX<<","<<std::isinf(grad_xt)<<","<<std::isinf(grad_yt)<<"\n";
		//ロボットの命令速度算出
		float w,v;
		set_command_vel(xrit,v0,v,w,th_t0);
		//std::cout<<"v,w,th_t0:"<<v<<","<<w<<","<<th_t0<<"\n";
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t0=th_t0+w*mv_t;
		xrft.x=xrft.x + l*cos(th_t0);
		xrft.y=xrft.y + l*sin(th_t0);
		//障害物の移動
		//ROS_INFO("move_obstacle_data...while\n");
		move_obstacle_data(mv_t);
		//debug
		//ROS_INFO("set_pub_mpc_debug_images()\n");
		//set_pub_mpc_debug_images(xrit);
		
		/*
		if(tr==time_range){
		
			std::cout<<"in cost:grad(x,y,w):"<<grad_xt<<","<<grad_yt<<","<<w<<"\n";
			std::cout<<"xrit:"<<xrit<<"\n";
			std::cout<<"pot_xrit:"<<get_pot_xt(xrit)<<"\n";
			std::cout<<"pot(x0,x1),(y0,y1):("<<pot_x0<<","<<pot_x1<<"),("<<pot_y0<<","<<pot_y1<<")\n";
			pot_maptt=pot_mapt.clone();
		
			std::cout<<"w in mpc:"<<w<<"\n";
			
		}
		*/
		
		//rate.sleep();
		tr-=mv_t;
	}
	/*
	cv::Point2f del=xrft-xrft0;
	vrate=std::abs(del.x)+std::abs(del.y);
	if(sum_cost>0){
		sum_cost/=vrate;
	}
	else{
		sum_cost*=vrate;
	}
	*/	
	return sum_cost;
}

float APF_MPC::get_speed(const cv::Point2f& xrft0,const float& vrt00)
{
	cv::Point2f xrft=xrft0;
	cv::Point2i xrit;
	float vrt0=vrt00;//
	float delta_v=0.01;
	float vrt1=vrt00+delta_v;
	//
	float max_v=0.3;
	float min_v=0.1;
	//
	double cost0=0;
	double cost1=0;
	float time_range=10;
	//
	float opt_v;
	//
	//std::cout<<"vrt0:"<<vrt0<<"\n";
	if(vrt1>=max_v){
		vrt0=vrt00-delta_v;
		vrt1=vrt00;//+delta_v;
	}
	//Process Once
	/*
	trans_point_f_to_i(xrft,xrit);
	if(xrit.x==xgi.x && xrit.y==xgi.y)
	{
		std::cout<<"Goal\n";
		return 0;
	}
	*/
	//std::cout<<"xrft0,xrft:"<<xrft0<<","<<xrft<<"\n";
	//ROS_INFO("culc_cost0...\n");
	cost0=culc_cost(xrft,vrt0,time_range);
	//ROS_INFO("culc_cost1...\n");
	cost1=culc_cost(xrft,vrt1,time_range);
	//predict param
	int search_num=20;
	float pot_th=0.10;//10%
	float pot_rate;
	//gradient v
	float grad_v;
	//
	bool flag01=false;
	bool flag10=false;
	//ROS_INFO("while...\n");
	while(search_num-->0&&ros::ok()){
		grad_v=(cost1-cost0)/delta_v;
		//
		/*
		pot_rate=std::abs((cost1-cost0)/cost0);
		std::cout<<"pot_rate:"<<pot_rate<<"\n";
		if(pot_rate<pot_th)
		{
			break;
		}
		*/
		//保留
		//if(grad_v>0){//cost0<cost1
		//std::cout<<"vrt("<<vrt0<<","<<vrt1<<")\n";
		//std::cout<<"cost("<<cost0<<","<<cost1<<")\n";
		std::cout<<"opt_v("<<opt_v<<")\n";
		if(cost0<=cost1){
			if(vrt0>=max_v){
				return max_v;
			}
			if(vrt0<=min_v){
				return min_v;
			}
			//ROS_INFO("vrt0<vrt1:(%f,%f)\n",vrt0,vrt1);
			opt_v=vrt0;
			vrt1=vrt0;
			cost1=cost0;
			vrt0=vrt0-delta_v;
			cost0=culc_cost(xrft,vrt0,time_range);
			flag01=true;
		}
		else{//cost1<cost0:vrt1<vrt0
			if(vrt1>=max_v){
				return max_v;
			}
			if(vrt1<=min_v){
				return min_v;
			}
			//ROS_INFO("vrt0>vrt1:(%f,%f)\n",vrt0,vrt1);
			opt_v=vrt1;
			vrt0=vrt1;
			cost0=cost1;
			vrt1=vrt0+delta_v;
			cost1=culc_cost(xrft,vrt1,time_range);	
			flag10=true;
		}
		if(flag01&&flag10)
			break;
				
	}
	return opt_v;
}


