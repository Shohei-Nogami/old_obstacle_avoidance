#include"img_prc_cls.h"

bool wf_f=false;
bool wfo_f=false;//true;//
int wfo_c=0;
const int wfo_cmax=10;
bool wfo_cf=true;
const int th_dis_bias=2;
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
//	//std::cout<<"count\n";
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
//	w_w=-(vr-vl)/d;//回転角速度
//	w_dyaw=w_w*dt;//回転角
//	//std::cout<<"wh:(w,dyaw):"<<"("<<w_w<<","<<w_dyaw<<")\n";
//	//std::cout<<"vs:(w,dyaw):"<<"("<<dyaw/dt<<","<<dyaw<<")\n";
//	w=dyaw;
//culc jacobi
//odometry 

//
	float X,Y;
	cv::Point2f ppt;
	for(int j=0;j<pts.size();j++){
		X=(float)(pts[j].x-width/2.0);//-width;
		Y=(float)(pts[j].y-height/2.0);//-height;
		float value_x;
		float value_y;
		ppt.x=pts[j].x- (float)(
		  	dx/pz[j]-X/pz[j]*dz
		  	-(f+pow(X,2.0)/f)*dyaw
		  	);
		if(std::isnan(ppt.x)){
			//std::cout<<"ppt.x is nan\n";
			//std::cout<<"pts.x,pz:"<<pts[j].x<<","<<pz[j]<<"\n";
		}
		ppt.y=pts[j].y-(float)(
			  	-(Y/pz[j]*dz)
			  	-(X*Y*dyaw/f
			  	));
		jnpts.push_back(ppt) ;
	}
/*
	for(int j=0;j<pts.size();j++){
		X=(float)(pts[j].x-width/2.0);//-width;
		Y=(float)(pts[j].y-height/2.0);//-height;
		ppt.x=pts[j].x- (float)(
		  	dx/pz[j]-X/pz[j]*w_w*dt
		  	-(1+pow(X,2.0)/f)*w_dyaw
		  	);
		ppt.y=pts[j].y-(float)(
			  	-(Y/pz[j]*w_w*dt)
			  	-(X*Y*w_dyaw/f
			  	));
		jnpts.push_back(ppt) ;
	}
*/
	//std::cout<<"insert\n";// bagu from here
	try{
		//std::cout<<"npts,jnpts:"<<npts.size()<<","<<jnpts.size()<<"\n";
		npts.insert(npts.end(),jnpts.begin(),jnpts.end());
		//std::cout<<"1";
	//---オプティカルフローを得る-----------------------------
		cv::calcOpticalFlowPyrLK(PreLgray,Lgray, pts, npts, sts, ers, cv::Size(21,21), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);
	//Delete the point that not be matched
		//std::cout<<"2";
		float pnz;
		for(int i=0;i<pts.size();i++){
			//std::cout<<"2.5 ";
			float depth_np=depth_img.at<float>(npts[i].y,npts[i].x);
			//std::cout<<"3";
			if(sts[i]&&!std::isnan(depth_np)&&!std::isinf(depth_np)&&depth_np>=0.5){
			//std::cout<<"4";
				points.push_back(pts[i]);
			//std::cout<<"5";
				newpoints.push_back(npts[i]);
			//std::cout<<"6";
				z.push_back(pz[i]);
			//std::cout<<"7";
				jnewpoints.push_back(jnpts[i]);
			//std::cout<<"8";
				nz.push_back(depth_img.at<float>(npts[i].y,npts[i].x));
			//std::cout<<"9";
				mpf.push_back(pmpf[i]);
			//std::cout<<"10\n";
			//std::cout<<"i,pts.size():"<<i<<","<<pts.size()<<"\n";
			}
		}
		//std::cout<<"pushback\n"; // bagu so far 
	}//try
/*	catch(const std::exception& ex) {
		std::cerr << ex.what() << std::endl;
	}//catch
*/	catch(cv::Exception& e){
		std::cerr << e.what() << std::endl;
	}
	
	for(int j=0;j<points.size();j++){
//----矢印描写---
		float L1=std::abs(newpoints[j].x-jnewpoints[j].x);//sqrt(z[j]);
		float L2=sqrt((newpoints[j].x-jnewpoints[j].x)*(newpoints[j].x-jnewpoints[j].x)
			+(newpoints[j].y-jnewpoints[j].y)*(newpoints[j].y-jnewpoints[j].y));//sqrt(z[j]);
//newpoints==points+LK
//jnewpoints==points+jacobi
//newpoints-jnewpoints==LK-jacobi
//2*points-jnewpoint==points-jacobi
		double th_optt=th_opt;
//		if(std::abs(dyaw)>0.01)
//			th_optt=th_opt*(std::abs(dyaw)/0.01);
//	//std::cout<<"th:"<<th_optt<<"\n";
		
		if(L1<th_optt/z[j]+0.5){
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point((int)points[j].x,
					(int)points[j].y),
				cv::Point((int)(newpoints[j].x-jnewpoints[j].x+points[j].x),
					(int)(newpoints[j].y-jnewpoints[j].y+points[j].y)),
				cv::Scalar(255,255,255));//白
				mpf[j]=0;
		}
		else{
			
			if(mpf[j]>=mth){
			  ImageProcesser::cvArrow(&Limg_view,
				cv::Point((int)points[j].x,
					(int)points[j].y),
				cv::Point((int)(newpoints[j].x-jnewpoints[j].x+points[j].x),
					(int)(newpoints[j].y-jnewpoints[j].y+points[j].y)),
				cv::Scalar(0,255,0));//緑
				mov.push_back((newpoints[j].x-jnewpoints[j].x)*z[j]/f);
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
			mpf[j]++;
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
//		//std::cout<<"delta flow:"<<newpoints[j].x-jnewpoints[j].x<<"\n";
	}
	double mov_sum=0;
	for(int i=0;i<mov.size();i++)
		mov_sum+=mov[i];
	std::cout<<"mov:"<<mov_sum/(int)mov.size()<<"\n";
	wfo_c++;
	wfo_cf=true;
}
