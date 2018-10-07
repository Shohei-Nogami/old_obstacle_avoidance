#include<apf.h>

void APF::set_command_vel(cv::Point2i& xri0,float& v,float& w)
{
		float vx = grad_map[0].at<float>(xri0.y,xri0.x);
		float vy = grad_map[1].at<float>(xri0.y,xri0.x);
		float v0 = std::sqrt(vx*vx+vy*vy);
		vx /= v0;
		vy /= v0;
		//反時計回りを正(0~360)
		float th = std::atan2(vy,vx);
				float delta_th;
		//-180<th_t<180
		if(th_t>M_PI)
		{
			th_t-=2*M_PI;
		}
		else if(th_t<-M_PI)
		{
			th_t+=2*M_PI;
		}
		
		if(th<0&&th_t>0)// 2*M_PI > th > M_PI && 0 < th_t < M_PI
		{
			delta_th= th + 2*M_PI - th_t;
		}
		else if(th>0&&th_t<0)
		{
			delta_th= th - (th_t+2*M_PI);
		}
		else
		{
			delta_th= th -th_t;
		}
		delta_th=th-th_t;
		if(delta_th<-M_PI)
		{
			delta_th+=2*M_PI;
		}
		else if(delta_th>M_PI)
		{
			delta_th-=2*M_PI;
		}
		//角速度(P制御)
		float Kp=1;
		w=Kp*delta_th;
		if(w>max_w)
		{
			w=max_w;
		}
		else if(w<-max_w)
		{
			w=-max_w;
		}
		//速度可変
		//float d=0.138;
		//float dif_v = w*2*d;
		//v=vrt-std::abs(dif_v)/2;
		//速度は一定
		v=vrt;
		
}
void APF::draw_path_mat(void)
{
	ros::NodeHandle n;
	ros::Rate rate(100);
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

		//ロボットの命令速度算出
		float w,v;
		set_command_vel(xri,v,w);
		std::cout<<"v,w,th_t:"<<v<<","<<w<<","<<th_t<<"\n";
		//ロボットの移動
		//mv_t:移動時間
		float l=v*mv_t;
		th_t=th_t+w*mv_t;
		xrf.x=xrf.x + l*cos(th_t);
		xrf.y=xrf.y + l*sin(th_t);
		
		//debug
		set_pub_debug_images();
		rate.sleep();
				
	}
}

