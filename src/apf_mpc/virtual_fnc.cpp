#include<apf_mpc.h>


void APF_MPC::set_command_vel(const cv::Point2i& xri0,const float& v0,float& v,float& w,float& th_t0)
{
	float vx = grad_xt;
	float vy = grad_yt;
	float v00 = std::sqrt(vx*vx+vy*vy);
	vx /= v00;
	vy /= v00;
	//反時計回りを正(0~360)
	//std::cout<<"grad_xt,yt:"<<grad_xt<<","<<grad_yt<<"\n";
	float th = std::atan2(vy,vx);
	float delta_th;
	//-180<th_t<180
	if(th_t0>M_PI)
	{
		th_t0-=2*M_PI;
	}
	else if(th_t0<-M_PI)
	{
		th_t0+=2*M_PI;
	}
	
	if(th<0&&th_t0>0)// 2*M_PI > th > M_PI && 0 < th_t < M_PI
	{
		delta_th= th + 2*M_PI - th_t0;
	}
	else if(th>0&&th_t0<0)
	{
		delta_th= th - (th_t0+2*M_PI);
	}
	else
	{
		delta_th= th -th_t0;
	}
	delta_th=th-th_t0;
	if(delta_th<-M_PI)
	{
		delta_th+=2*M_PI;
	}
	else if(delta_th>M_PI)
	{
		delta_th-=2*M_PI;
	}
	//limitを速度の大きさと設定
	//set_command_limit(v0/2);
	
	//角速度(P制御)
	float Kp=1;
	w=Kp*delta_th;
	//std::cout<<"th,th_t0:"<<th<<","<<th_t0<<"\n";
	//std::cout<<"delta_th,w:"<<delta_th<<","<<w<<"\n";
	if(w>max_w)
	{
		w=max_w;
	}
	else if(w<-max_w)
	{
		w=-max_w;
	}
	//速度可変
	float d=0.138;
	float dif_v = w*2*d;
	//v=v0-std::abs(dif_v)/2;
	//速度可変
	v=v0;
	
}
