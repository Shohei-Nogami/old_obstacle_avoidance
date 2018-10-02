#include<apf.h>

float APF::culc_dis(const int& w0,const int& h0,const int& w1,const int& h1)
{
	float del_w=(float)(w0-w1)*reso;
	float del_h=(float)(h0-h1)*reso;
	return std::sqrt( del_w*del_w + del_h*del_h );
}
float APF::culc_fr(const float& dis,const int& obstNum)
{
	float weight=0.01;
	float safe_rate=3;
	float margin=0.1;
	float eps=0.1;
	
	for(int i=0;i<safe_rate;i++)
	{	
		if(dis>(rr+cr+margin)*(safe_rate-i)){
			return (1/ (dis+eps))*weight*i;
		}
	}
	if(dis>(rr+cr)){
		return (1/ (dis+eps))*max_pot/2*obstNum;
	}
	
	/*
	if(dis>(rr+cr+margin)){
		return (1/ (dis+eps))*weight;
	}
	*/
	return ((1/ (dis+eps))*max_pot*obstNum);
}
float APF::culc_fr(const int& w0,const int& h0,const int& w1,const int& h1,const int& obstNum)
{
	float del_w=(float)(w0-w1)*reso;
	float del_h=(float)(h0-h1)*reso;
	float dis=std::sqrt( del_w*del_w + del_h*del_h );
	float weight=0.01;
	float safe_rate=2;
	float danger_rate=2;
	float margin=0.1;
	float eps=0.1;
	for(int i=0;i<safe_rate;i++)
	{	
		if(dis>(rr+cr+margin)*(safe_rate-i)){
			return (1/ (dis+eps))*weight*i*danger_rate;
		}
	}
	if(dis>(rr+cr)){
		return (1/ (dis+eps))*max_pot/2*obstNum;
	}
	return ((1/ (dis+eps))*max_pot*obstNum);
}
//L1
float APF::culc_L1dis(const int& w0,const int& h0,const int& w1,const int& h1)
{
	float del_w=(float)(w0-w1)*reso;
	float del_h=(float)(h0-h1)*reso;
	return std::abs(del_w) + std::abs(del_h);
}
float APF::culc_L1fr(float& dis,const int& obstNum)
{
	float weight=0.02;
	float safe_rate=3;
	float danger_rate=10;
	float margin=0.2;
	float root2=1.4142;
	float eps=0.1;
	for(int i=0;i<safe_rate;i++)
	{
		if(dis>( (rr+cr+margin)/root2*2) *(safe_rate-i)){
			return (1/ (dis+eps))*weight*i*danger_rate;
		}
	}
	if(dis>(rr+cr)/root2*2){
		return (1/ (dis+eps))*max_pot/2*obstNum;
	}
	return ((1/ (dis+eps))*max_pot*obstNum);
}
//Chev
float APF::culc_chvdis(const int& w0,const int& h0,const int& w1,const int& h1)
{
	float del_w=(float)(w0-w1)*reso;
	float del_h=(float)(h0-h1)*reso;
	return del_w > del_h ? del_w : del_h;
}
float APF::culc_chvfr(const int& w0,const int& h0,const int& w1,const int& h1,const int& obstNum)
{
	float del_w=(float)std::abs(w0-w1)*reso;
	float del_h=(float)std::abs(h0-h1)*reso;
	float dis= del_w > del_h ? del_w : del_h;
	float weight=0.02;
	float safe_rate=2;
	float danger_rate=2;
	float margin=0.2;
	float eps=0.1;
	for(int i=0;i<safe_rate;i++)
	{	
		if(dis>(rr+cr+margin)*(safe_rate-i)){
			return (1/ (dis+eps))*weight*i*danger_rate;
		}
	}
	if(dis>(rr+cr)){
		return (1/ (dis+eps))*max_pot/2*obstNum;
	}
	return ((1/ (dis+eps))*max_pot*obstNum);
}
//
float APF::culc_fa(float dis)
{
	float weight=1;
	float eps=0.1;
	if(1/(dis+eps)>max_pot)
	{ 
		return (-max_pot);
	}
	else
	{
		return (-1/((dis+eps)) )*weight;	
	}
}
float APF::culc_fa(const int& w0,const int& h0,const int& w1,const int& h1)
{
	float del_w=(float)(w0-w1)*reso;
	float del_h=(float)(h0-h1)*reso;
	float dis=std::sqrt( del_w*del_w + del_h*del_h );
	float weight=1;
	float eps=0.1;
	if(1/(dis+eps)>max_pot)
	{ 
		return (-max_pot);
	}
	else
	{
		return (-1/(dis+eps) )*weight;	
	}
}
void APF::create_pot_map(void){
	int W=map_wi;
	int H=map_hi;
	float sum_potf=0;
	float sum_pota=0;
	//
	int ch_g = grid_map.channels();
	int ch_p = pot_map.channels();
	//
	search_obst_pt();
	int obst_num=(int)obst_pti.size();
	//
	for(int h0=0;h0<H;h0++){
		float *ppot = pot_map.ptr<float>(h0);
		for(int w0=0;w0<W;w0++){
			//斥力算出
			bool break_flag=false;
			/*
			for(int h=0;h<H;h++){
				uint8_t *pgrid = grid_map.ptr<uint8_t>(h);
				for(int w=0;w<W&&ros::ok();w++){
					
					if(h0==h&&w0==w)
					{
						continue;
					}
					//if(grid_map.at<uint8_t>(h,w)>=1)
					if(pgrid[w * ch_g]>=1)
					{
						//L2
						float dis=culc_dis(w0,h0,w,h);
						//pot_map.at<float>(h0,w0)+=culc_fr(dis,obst_num);
						ppot[w0 * ch_p]+=culc_fr(dis,obst_num);
						if(dis<(rr+cr)){
							break_flag=true;
							break;
						}
						
					}					
				}
				if(break_flag)
					break;
			}
			*/
			
			for(int k=0;k<obst_pti.size();k++)
			{
				float dis=culc_dis(w0,h0,obst_pti[k].x,obst_pti[k].y);
				//pot_map.at<float>(h0,w0)+=culc_fr(dis,obst_num);
				if(dis<(rr+cr)){
					ppot[w0 * ch_p]=culc_fr(dis,obst_num);
					break_flag=true;
					break;
				}
				ppot[w0 * ch_p]+=culc_fr(dis,obst_num);
				
			}
			
			if(break_flag)
				continue;
			if(obst_num>0)
			{
				//pot_map.at<float>(h0,w0)/=obst_num;
				ppot[w0 * ch_p]/=obst_num;
			}
			//引力算出
			//std::cout<<"fa:"<<-culc_fa(w0,h0,xgi.x,xgi.y)<<"\n";
			
			//pot_map.at<float>(h0,w0)+=culc_fa(culc_dis(w0,h0,xgi.x,xgi.y));
			ppot[w0 * ch_p]+=culc_fa(culc_dis(w0,h0,xgi.x,xgi.y));
			//sum_pot+=pot_map.at<float>(h0,w0);
			//std::cout<<"pot_map,sum_pot:"<<pot_map.at<float>(h0,w0)<<","<<sum_pot<<"\n";
		}
	}
}
void APF::search_obst_pt(void)
{
	int W=map_wi;
	int H=map_hi;
	obst_pti.resize(W*H);
	int ch_g = grid_map.channels();
	int vsize=0;
	for(int h=0;h<H;h++){
		uint8_t *pgrid = grid_map.ptr<uint8_t>(h);
		for(int w=0;w<W&&ros::ok();w++){
			if(pgrid[w * ch_g]>=1)
			{
				obst_pti[vsize++]=cv::Point2i(w,h);
			}
		}	
	}
	obst_pti.resize(vsize);
	
}


