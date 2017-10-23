#include"img_prc_cls.h"

bool wf_f=false;
bool wfo_f=true;//
int wfo_c=0;
const int wfo_cmax=10;
bool wfo_cf=true;
const int th_dis_bias=5;
const int th_count=5;
//画像フレーム取得後呼び出される関数
void ImageProcesser::imageProcess()
{
	//debug
//	ROS_INFO("process start");

	Limg_view=Limg.clone();//imshow用のMat
	if(!isPrevimage())
		return ;

//グレースケール化
//	cv::Mat Lgray,PreLgray;
	cv::cvtColor(Limg,Lgray,CV_BGR2GRAY);
	cv::cvtColor(PreLimg,PreLgray,CV_BGR2GRAY);
//参照URL:http://opencv.jp/opencv-2svn/cpp/motion_analysis_and_object_tracking.html#cv-calcopticalflowpyrlk

	for(int i=0;i<cn;i++){
		for(int j=0;j<cn;j++){
			clp_img[i][j]=PreLgray(cv::Rect((int)(j*(width)/cn),(int)(i*(height)/cn),(int)(width/cn),(int)(height/cn)));
		}
	}
	count_feature_points();
	add_feature_points();
	if(!pts.size()){
		return ;
	}
	//memory release
//	PreLgray.release();
//	Lgray.release();
//-----画像ヤコビアンを用いて--------------------
//-----ロボットの移動によるnewpointsの求める-----
//add img_prc_cls.h
//  std::vector<cv::Point2i> jnpts;
//  std::vector<cv::Point2i> jnewpoints;
//  static const double vr;
//  static const double vl;
//  static const double d=0.276;//車輪幅
//  static const double w_w;
//  double w_dyaw;
//add vector_function.cpp
//in reserve_vectors
//  jnpts.reserve(point_size);
//  jnewpoints.reserve(point_size);
//in clear_vectors
//  jnpts.clear();
//  jnewpoints.clear();
//set param

	double dz=global_dx;//visual odometry z座標
	double dx=global_dy;//visual odometry x座標
	w_w=(vr-vl)/d;//回転角速度
	w_dyaw=w_w*dt;//回転角
	
//	w=dyaw;
//culc jacobi
	float X,Y;
	cv::Point2f ppt;
	for(int j=0;j<pts.size();j++){
		X=(float)(pts[j].x-width/2.0);//-width;
		Y=(float)(pts[j].y-height/2.0);//-height;
		float value_x;
		float value_y;
		ppt.x=pts[j].x- (float)(
		  	dx/pz[j]-X/pz[j]*dz
		  	-(1+pow(X,2.0)/f)*dyaw
		  	);
		ppt.y=pts[j].y-(float)(
			  	-(Y/pz[j]*dz)
			  	-(X*Y*dyaw/f
			  	));
		jnpts.push_back(ppt) ;
	}
	npts.insert(npts.end(),jnpts.begin(),jnpts.end());
//---オプティカルフローを得る-----------------------------
	cv::calcOpticalFlowPyrLK(PreLgray,Lgray, pts, npts, sts, ers, cv::Size(21,21), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);
//Delete the point that not be matched
	float pnz;
	for(int i=0,k=0;i<pts.size();i++){
		if(sts[i]&&!std::isnan(depth_img.at<float>(npts[i].y,npts[i].x))){
			points.push_back(pts[i]);
			newpoints.push_back(npts[i]);
			z.push_back(pz[i]);
			jnewpoints.push_back(jnpts[i]);
			nz.push_back(depth_img.at<float>(npts[i].y,npts[i].x));
		}
	}
//remove bias noise of jacobi
//case1
/*	if(points.empty())
		std::cout<<"empty points\n";
	double sum_bias=0;
	double ave_bias;
//	std::vector<double> p_bias;//img_prc_cls.h
	//culculate average bias
	for(int j=0;j<points.size();j++){
		sum_bias+=newpoints[j].x-jnewpoints[j].x;
	}
	ave_bias=sum_bias/(int)points.size();
	std::cout<<"ave_bias1:"<<ave_bias<<"\n";
	//Removal Outliers
	for(int j=0;j<points.size();j++){
		if(std::abs(newpoints[j].x-points[j].x-ave_bias) < std::abs(ave_bias)/2)
			p_bias.push_back(newpoints[j].x-points[j].x);
	}
	//culculate average bias again
	sum_bias=0;
	for(int j=0;j<p_bias.size();j++){
		sum_bias+=p_bias[j];
	}
	if(p_bias.size()!=0)
		ave_bias=sum_bias/(int)p_bias.size();
	std::cout<<"ave_bias2:"<<ave_bias<<"\n";
	if(std::abs(ave_bias)>50)
		show_speed();
	if(std::isnan(ave_bias))
		std::cout<<"ave,sum,pbias_size:"<<ave_bias<<","<<sum_bias<<","<<(int)p_bias.size()<<"\n";
	//Removal average bias
	for(int j=0;j<points.size();j++){
		newpoints[j].x=newpoints[j].x-ave_bias;
	}
*/
//case2
	double sum_bias=0;
	double sum_dis=0;
	double ave_bias,dis_bias;//dis_bias is sqrt(Dispersion)
	int fcount=0;
//	std::vector<double> p_bias,p_bias0;//img_prc_cls.h
	//culculate average bias
	for(int j=0;j<points.size();j++){
		sum_bias+=newpoints[j].x-jnewpoints[j].x;
	}
	ave_bias=sum_bias/points.size();
	//culculate Dispersion
	for(int j=0;j<points.size();j++){
		sum_dis+=(newpoints[j].x-points[j].x-ave_bias)*(newpoints[j].x-points[j].x-ave_bias);
	}
	dis_bias=sqrt(sum_dis/points.size());
	for(int j=0;j<points.size();j++){
		if(std::abs(newpoints[j].x-points[j].x) < std::abs(ave_bias)+dis_bias)
			p_bias0.push_back(newpoints[j].x-points[j].x);
	}
	while(ros::ok()&&fcount++<th_count){
		sum_bias=0;
		//culc sum
		for(int j=0;j<p_bias0.size();j++){
			sum_bias+=newpoints[j].x-jnewpoints[j].x;
		}
		//culc ave
		ave_bias=sum_bias/p_bias0.size();
		sum_dis=0;
		//culculate Dispersion
		for(int j=0;j<p_bias0.size();j++){
			sum_dis+=(p_bias0[j]-ave_bias)*(p_bias0[j]-ave_bias);
		}
		dis_bias=sqrt(sum_dis/p_bias0.size());
		//Dispersion < threshold -> break
		if(dis_bias<th_dis_bias)
			break;
		//Removal Outliers
		for(int j=0;j<p_bias0.size();j++){
			if(std::abs(newpoints[j].x-points[j].x) < std::abs(ave_bias)+dis_bias)
				p_bias.push_back(newpoints[j].x-points[j].x);
		}
		p_bias0.clear();
		//renew p_bias0 and clear p_bias
		p_bias0.insert(p_bias0.end(),p_bias.begin(),p_bias.end());
		p_bias.clear();
	}
	//remove bias
	for(int j=0;j<points.size();j++)
		newpoints[j].x=newpoints[j].x-ave_bias;
	


	for(int j=0;j<points.size();j++){
//----矢印描写---
		float L1=std::abs(newpoints[j].x-jnewpoints[j].x)*sqrt(z[j]);
//newpoints==points+LK
//jnewpoints==points+jacobi
//newpoints-jnewpoints==LK-jacobi
//2*points-jnewpoint==points-jacobi
		double th_optt=th_opt;
//		if(std::abs(dyaw)>0.01)
//			th_optt=th_opt*(std::abs(dyaw)/0.01);
//	std::cout<<"th:"<<th_optt<<"\n";
		if(L1<th_optt){
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point((int)points[j].x,
					(int)points[j].y),
				cv::Point((int)(newpoints[j].x-jnewpoints[j].x+points[j].x),
					(int)(newpoints[j].y-jnewpoints[j].y+points[j].y)),
				cv::Scalar(255,255,255));//白
		}
		else{
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point((int)points[j].x,
					(int)points[j].y),
				cv::Point((int)newpoints[j].x,
					(int)newpoints[j].y),
				cv::Scalar(0,200,200));//黄
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point((int)points[j].x
					,
					(int)points[j].y),
				cv::Point((int)(2*points[j].x-jnewpoints[j].x),
					(int)(2*points[j].y-jnewpoints[j].y)),
				cv::Scalar(200,0,200));//紫
		}

//output file
		if(wf_f){
			std::ofstream ofss("./Documents/output_opticalflow.csv",std::ios::app);
			ofss<<points[j].x-width/2<<","//X
				<<points[j].y-height/2<<","//Y
				<<z[j]<<","//z
				<<dx<<","//dx
				<<dz<<","//dz
				<<dyaw<<","//dw
				<<dt<<","//dt
				<<","
				<<newpoints[j].x-points[j].x<<","//観測x
				<<newpoints[j].y-points[j].y<<","//観測y
				<<jnewpoints[j].x-points[j].x<<","//ヤコビx
				<<jnewpoints[j].y-points[j].y<<","//ヤコビy
				<<","
				<<newpoints[j].x-jnewpoints[j].x<<","//観測x-jacobi
				<<newpoints[j].y-jnewpoints[j].y<<","//観測y-jacobi
				<<std::endl;
			}
		if(wfo_f){
			if(wfo_c>=wfo_cmax)
				wfo_f=false;
			std::string c= std::to_string(wfo_c);
			std::string ofilename="./Documents/output_opticalflow"+c+".csv";
			if(wfo_cf){
				std::ofstream ofss(ofilename,std::ios::app);
				ofss<<"X"<<","//X
					<<"Y"<<","//Y
					<<"z"<<","//z
					<<"dx"<<","//dx
					<<"dz"<<","//dz
					<<"dyaw"<<","//dw
					<<"dt"<<","//dt
					<<","
					<<"観測x"<<","//観測x
					<<"観測y"<<","//観測y
					<<"ヤコビx"<<","//ヤコビx
					<<"ヤコビy"<<","//ヤコビy
					<<","
					<<"観測x-ヤコビx"<<","//観測x-jacobi
					<<"観測y-ヤコビy"<<","//観測y-jacobi
					<<std::endl;
					wfo_cf=false;
			}
			std::ofstream ofss(ofilename,std::ios::app);
			ofss<<points[j].x-width/2<<","//X
				<<points[j].y-height/2<<","//Y
				<<z[j]<<","//z
				<<dx<<","//dx
				<<dz<<","//dz
				<<dyaw<<","//dw
				<<dt<<","//dt
				<<","
				<<newpoints[j].x-points[j].x<<","//観測x
				<<newpoints[j].y-points[j].y<<","//観測y
				<<jnewpoints[j].x-points[j].x<<","//ヤコビx
				<<jnewpoints[j].y-points[j].y<<","//ヤコビy
				<<","
				<<newpoints[j].x-jnewpoints[j].x<<","//観測x-jacobi
				<<newpoints[j].y-jnewpoints[j].y<<","//観測y-jacobi
				<<std::endl;
		}
//		std::cout<<"delta flow:"<<newpoints[j].x-jnewpoints[j].x<<"\n";
	}
	wfo_c++;
	wfo_cf=true;
}
