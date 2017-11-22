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

	for(int i=0;i<cnh;i++){
		for(int j=0;j<cnw;j++){
			clp_img[i][j]=PreLgray(cv::Rect((int)(j*(width)/cnw),(int)(i*(height)/cnh),(int)(width/cnw),(int)(height/cnh)));
		}
	}
//	//std::cout<<"count\n";
	count_feature_points();
	add_feature_points();
	if(!pts.size()){
		return ;
	}
//

//修正必要
//	std::cout<<"dz:"<<dz<<"\n";
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
		ppt.y=pts[j].y-(float)(
				-(Y/pz[j]*dz)
				-(X*Y*dyaw/f
				));
		jnpts.push_back(ppt) ;
	}
	npts.insert(npts.end(),jnpts.begin(),jnpts.end());
//---オプティカルフローを得る-----------------------------
	cv::calcOpticalFlowPyrLK(PreLgray,Lgray, pts, npts, sts, ers, cv::Size(15,15), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);

	float pnz;
	for(int i=0;i<pts.size();i++){
		float depth_np=depth_img.at<float>(npts[i].y,npts[i].x);
		if(sts[i]&&!std::isnan(depth_np)&&!std::isinf(depth_np)&&depth_np>=0.5){
			points.push_back(pts[i]);
			newpoints.push_back(npts[i]);
			z.push_back(pz[i]);
			jnewpoints.push_back(jnpts[i]);
			nz.push_back(depth_np);
			nmk.push_back(mk[i]);
			npk.push_back(pk[i]);
		}
	}
//kalman filter
	for(int j=0;j<points.size();j++){
		double ver_Uk=0.6;
		double ver_tk=0.01;
		double fk_data[2][2]={ {1,-dyaw/dt},{dyaw/dt,1} };	
		cv::Mat fk(2,2,CV_64FC1,fk_data);
		double gk_data[2][3]={ {-1,0,pz[j]},{0,-1,-f*points[j].x/z[j]} };
		cv::Mat gk(2,3,CV_64FC1,gk_data);
		double wk_data[3][1]={ {dx/dt},{dz/dt},{dyaw/dt} };
		cv::Mat wk(3,1,CV_64FC1,wk_data);
		double hk_data[2][3]={ {f/pz[j],0,-f*points[j].x/(pz[j]*z[j])},{0,f/pz[j],-f*points[j].y/(z[j]*z[j])} };//provisional 
		cv::Mat hk(2,3,CV_64FC1,hk_data);
		double Uk_data[2][2]={ {ver_Uk,0},{0,ver_Uk}};
		cv::Mat Uk(2,2,CV_64FC1,Uk_data);
		double Wk_data[3][3]={{z[j]*z[j]*ver_Uk*ver_tk*ver_tk,0,0},{0,z[j]*z[j]*ver_Uk*ver_tk*ver_tk,0},{0,0,z[j]*z[j]*ver_Uk*ver_tk*ver_tk}};
		cv::Mat Wk(3,3,CV_64FC1,Wk_data);
		double xk_t_1_data[2][1];
		cv::Mat xk_t_tld,xk_t_hat;
//		cv::Mat mk;
//		cv::Mat pk;
		if(npk[j].at<double>(0,0)!=0){
			nmk[j]=fk*npk[j]*fk.t()+gk*Wk*gk.t();
//			xk_t_1_data[2][1]={ {f*(~~~~~~~~~)/z[j]},{z[j]}};//tochuuu	
		}
		else{
//			xk_t_1_data[2][1]{ {f*points[j].x/z[j]},{z[j]}};				
		}
		npk[j]=(nmk[j].inv()+hk.t()*Uk.inv()*hk).inv();//.inv()<-?
//		npk[j]=nmk[j].inv()+hk.t()*Uk.inv()*hk;
//		npk[j]=npk[j].inv();
		cv::Mat xk_t_1(2,1,CV_64FC1,xk_t_1_data);
		
		//Estimation
		xk_t_tld=fk*xk_t_1+gk*wk;
		
		double xk_temp_data[2][1]={ {f*points[j].x/z[j]},z[j] };
		cv::Mat xk_temp(2,1,CV_64FC1,xk_temp_data);
		double uk_data[2][1]={ {f*points[j].x/z[j]},{f*points[j].y/z[j]} };
		cv::Mat uk(2,1,CV_64FC1,uk_data);
		xk_t_hat=xk_t_tld+npk[j]*hk.t()*Uk.inv()*(xk_temp-(hk*xk_t_tld+uk));
		
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

		ImageProcesser::cvArrow(&Limg_view,
				cv::Point((int)points[j].x,
					(int)points[j].y),
				cv::Point((int)(newpoints[j].x-jnewpoints[j].x+points[j].x),
					(int)(newpoints[j].y-jnewpoints[j].y+points[j].y)),
				cv::Scalar(0,200,200));//

			//detect moving area	



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
	}
	wfo_c++;
	wfo_cf=true;


}

