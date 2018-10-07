#include<apf_mpc.h>


//obstacle position:pt,robot position:xti
float APF_MPC::culc_mv_obstacle_fr(const cv::Point2i xti,const int& obstNum){
	float sum_mvfr=0;
	bool break_flag=false;
	for(int n=0;n<mv_obsts.size();n++){
		cv::Point2i pti;
		for(int k=0;k<mv_obsts[n].data.size();k++)
		{
			cv::Point2f pt=mv_obsts[n].data[k];
			pt.x+=mv_obsts[n].mvx;
			pt.y+=mv_obsts[n].mvy;
			pt.x+=mv_obsts[n].mvxt;
			pt.y+=mv_obsts[n].mvyt;
			if(trans_point(pt,pti))
			{
				float dis=culc_dis(xti.x,xti.y,pti.x,pti.y);
				sum_mvfr+=culc_fr(dis,obstNum);
				if(dis<(rr+cr)){
					break_flag=true;
					break;
				}
			}
			else{
				sum_mvfr+=0;
			}
		}
		if(break_flag){
			break;
		}
	}
	if(mv_data_size>0)
		sum_mvfr/=obstNum;
	return sum_mvfr;
	
}

void APF_MPC::add_mv_pot(const cv::Point2i xti,const int& obstNum){
	
	int W=map_wi;
	int H=map_hi;
	int delta=1;
	cv::Point2i yti0=xti;
	cv::Point2i xti0=xti;
	//ROS_INFO("culc_mv_obstacle_fr...add_mv_pot\n");
	//std::cout<<"xti:"<<xti<<"\n";
	pot_mapt.at<float>(xti.y,xti.x)+=culc_mv_obstacle_fr(xti,obstNum);
	//ROS_INFO("if(xti.y==0)...add_mv_pot\n");
	if(xti.y==0)
	{
		yti0.y+=delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);	
	}
	else if(xti.y==H-delta)
	{
		yti0.y-=delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);		
	}
	else{
		yti0.y=xti.y+delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);	
		yti0.y=xti.y-delta;
		pot_mapt.at<float>(yti0.y,yti0.x)+=culc_mv_obstacle_fr(yti0,obstNum);			
	}
	//ROS_INFO("if(xti.x==0)...add_mv_pot\n");
	if(xti.x==0)
	{
		xti0.x+=delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
	}
	else if(xti.x==W-delta)
	{
		xti0.x-=delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
	}
	else
	{
		xti0.x=xti.x+delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
		xti0.x=xti.x-delta;
		pot_mapt.at<float>(xti0.y,xti0.x)+=culc_mv_obstacle_fr(xti0,obstNum);	
	}
}

