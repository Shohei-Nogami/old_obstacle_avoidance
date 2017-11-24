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
//	add_feature_points();

///////////////void ImageProcesser::add_feature_points(void)
	auto detector = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
	cv::Point2i ppts;
	float ptz;
	bool flag;
	for(int i=0;i<cnh;i++){
		for(int j=0;j<cnw;j++){
			if((int)cp[i][j].size()<th_clpimg){
				detector.detect(clp_img[i][j], keypoints);	
				for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
		 			itk != keypoints.end(); ++itk){
					flag=false;
				   	ppts.x=j*width/cnw+itk->pt.x;
					ppts.y=i*height/cnh+itk->pt.y;
					for(int k=0;k<pts.size();k++){
						if(std::abs(cp[i][j][k].x-ppts.x)<1&&std::abs(cp[i][j][k].y-ppts.y)<1){
					    	flag=true;
					    	break;
						}
					}
					if(flag)
						continue;
					ptz=Predepth.at<float>(
						ppts.y,
						ppts.x
						);
					if(!std::isnan(ptz)&&!std::isinf(ptz)&&ptz>=0.5&&(int)pts.size()<point_size){
						pts.push_back(ppts);
						pz.push_back(ptz);
//						double mk_data[3][3]={{5.0,0.0,0.0},{0.0,5.0,0.0},{0.0,0.0,5.0}};
//						mk.push_back(cv::Mat(3,3,CV_64FC1,mk_data));
						double sgm_data[3][3]={{5.0,0.0,0.0},{0.0,5.0,0.0},{0.0,0.0,5.0}};
						sgm_p.push_back(cv::Mat(3,3,CV_64FC1,sgm_data));
//						double pk_data[3][3]={{0,0,0},{0,0,0},{0,0,0}};
//						pk.push_back(cv::Mat(3,3,CV_64FC1,pk_data));
//						double pk_data[2][2]={{0,0},{0,0}};
//						pk.push_back(cv::Mat(2,2,CV_64FC1,pk_data));
//						std::cout<<"mk[0]:"<<mk[0]<<"\n";
						double xt_data[3][1]={{0},{0},{0}};
//						cv::Mat xk_temp(2,1,CV_64FC1,xk_data);
						xt_hat_p.push_back(cv::Mat(3,1,CV_64FC1,xt_data));
//						mk_once_flag.push_back(true);
//						mk_temp.release();
						xt_once_p.push_back(true);
						
					}
				}//std::cout<<"1mk[0]:"<<mk[0]<<"\n";
			}//std::cout<<"2mk[0]:"<<mk[0]<<"\n";
		}//std::cout<<"3mk[0]:"<<mk[0]<<"\n";
	}//std::cout<<"4mk[0]:"<<mk[0]<<"\n";
//	std::cout<<"mk.size:"<<mk.size()<<"\n";



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

				sgm.push_back(sgm_p[i]);
				if(xt_hat_p[i].at<double>(0,0)!=0){
					std::cout<<"xt_hat_p[i]:"<<xt_hat_p[i]<<"\n";
				}
				xt_hat.push_back(xt_hat_p[i]);
				xt_once.push_back(xt_once_p[i]);
			}
		}

//	std::cout<<"xk_hat[0]:"<<xk_hat[0]<<"\n";
//	std::cout<<"pk[0]:"<<pk[0]<<"\n";
//	std::cout<<"mk[0]:"<<mk[0]<<"\n";
//	std::cout<<"nxk_hat[0]:"<<nxk_hat[0]<<"\n";
//	std::cout<<"npk[0]:"<<npk[0]<<"\n";
//	std::cout<<"nmk[0]:"<<nmk[0]<<"\n";
//	pxk_hat.insert(pxk_hat.end(),nxk_hat.begin(),nxk_hat.end());
	for(int j=0;j<points.size();j++){
	//観測方程式（量子化誤差を考慮しない）場合
/*		double ver_Uk=0.6;
		double ver_tk=0.01;
		std::cout<<"dyaw,dt,dyaw/dt:"<<dyaw<<","<<dt<<","<<dyaw/dt<<"\n";
		double fk_data[2][2]={ {1,-dyaw},{dyaw,1} };	
		cv::Mat fk(2,2,CV_64FC1,fk_data);
		std::cout<<"fk:"<<fk<<"\n";
		double gk_data[2][3]={ {-1,0,z[j]},{0,-1,-z[j]*(points[j].x-width/2)/f} };
		cv::Mat gk(2,3,CV_64FC1,gk_data);
		std::cout<<"gk:"<<gk<<"\n";
		double wk_data[3][1]={ {dx},{dz},{dyaw} };
		cv::Mat wk(3,1,CV_64FC1,wk_data);
		std::cout<<"wk:"<<wk<<"\n";
//		double Wk_data[3][3]={{z[j]*z[j]*ver_Uk*ver_tk*ver_tk,0,0},{0,z[j]*z[j]*ver_Uk*ver_tk*ver_tk,0},{0,0,z[j]*z[j]*ver_Uk*ver_tk*ver_tk}};
		double Wk_data[3][3]={{0.02,0,0},{0,0.02,0},{0,0,0.02}};
		cv::Mat Wk(3,3,CV_64FC1,Wk_data);
		std::cout<<"Wk:"<<Wk<<"\n";
		double xk_t_1_data[2][1]={ {(points[j].x-width/2)*z[j]/f},{z[j]}};//------------	;
		cv::Mat xk_t_1(2,1,CV_64FC1,xk_t_1_data);//------------
		cv::Mat xk_t_tld;//,xk_hat;
//		std::cout<<"xk_t_1:"<<xk_t_1<<"\n";//------------------------xk_t_1
		if(npk[j].at<double>(0,0)!=0){
			nmk[j]=fk*npk[j]*fk.t()+gk*Wk*gk.t();
			xk_t_1=nxk_hat[j];
//			std::cout<<"nmk[j]:"<<nmk[j]<<"\n";
//			std::cout<<"xk_t_1:"<<xk_t_1<<"\n";
		}
		double I_data[2][2]={ {1,0},{0,1} };
		cv::Mat I(2,2,CV_64FC1,I_data);
		npk[j]=(nmk[j].inv()+I).inv();
		std::cout<<"nmk:"<<nmk[j]<<"\n";
		//Estimation
		std::cout<<"fk:"<<fk<<"\n";
		std::cout<<"xk_t_1:"<<xk_t_1<<"\n";//-----------------------xk_t_1
		std::cout<<"fk*xk_t_1:"<<fk*xk_t_1<<"\n";
		xk_t_tld=fk*xk_t_1+gk*wk;
		std::cout<<"xk_t_tld:"<<xk_t_tld<<"\n";
//		double xk_temp_data[2][1]={ {(newpoints[j].x-width/2)*nz[j]/f},{nz[j]} };
		double xk_temp_data[2][1]={ {(newpoints[j].x-width/2)*nz[j]/f},{nz[j]} };
		cv::Mat xk_temp(2,1,CV_64FC1,xk_temp_data);		
		std::cout<<"xk_temp:"<<xk_temp<<"\n";
//		double uk_data[2][1]={ {(points[j].x-width/2)*z[j]/f},{z[j]}}//{-(points[j].y-height/2)*z[j]/f} };
		double uk_data[2][1]={ {(points[j].x-width/2)*z[j]/f},{z[j]}};
		cv::Mat uk(2,1,CV_64FC1,uk_data);
		std::cout<<"uk:"<<uk<<"\n";
		nxk_hat[j]=xk_t_tld+npk[j]*(xk_temp-(xk_t_tld+uk));
		std::cout<<"nxk_hat[j]"<<nxk_hat[j]<<"\n";
		double x_hat_data[2][1]={ {nxk_hat[j].at<double>(0,0)*f/nxk_hat[j].at<double>(1,0)+width/2},{points[j].y*nxk_hat[j].at<double>(1,0)/z[j]} };
		x_hat.push_back(cv::Mat(2,1,CV_64FC1,x_hat_data));
		
		std::cout<<"x_hat[j]:"<<x_hat[j]<<"\n";
		std::cout<<"points[j]:"<<points[j]<<"\n";
		std::cout<<"newpoints[j]:"<<newpoints[j]<<"\n";
*/
		std::cout<<"1\n";
		double rt_data[3][3]={ {0.05,0,0},{0,0.05,0},{0,0,0.05} };
		cv::Mat rt(3,3,CV_64FC1,rt_data);	 
		double qt_data[3][3]={ {0.05,0,0},{0,0.05,0},{0,0,0.05} };
		cv::Mat qt(3,3,CV_64FC1,qt_data);
//		double at_data[3][3]={ {1,droll,-dyaw},{-droll,1,dpitch},{dyaw,-dpitch,1} };		
		double at_data[3][3]={ {1,0,-dyaw},{0,1,0},{dyaw,0,1} };
		cv::Mat at(3,3,CV_64FC1,at_data);
//		double btut_data[3][1]={ {dx},{dy},{dz} };		
		double btut_data[3][1]={ {dx},{0},{dz} };
		cv::Mat btut(3,1,CV_64FC1,at_data);
		std::cout<<"2\n";
		if(xt_hat[j].at<double>(0,0)==0||xt_once[j]==true){
			double xt_0_data[3][1]={ {(points[j].x-width/2)*z[j]/f},{-(points[j].y-width/2)*z[j]/f},{z[j]} };
			cv::Mat xt_0(3,1,CV_64FC1,xt_0_data);
			xt_hat[j]=xt_0;
			std::cout<<"2.5\n";
			xt_once[j]=false;
		}
		std::cout<<"3\n";
		std::cout<<"xt_hat[j]:"<<xt_hat[j]<<"\n";
		cv::Mat atxt_temp=at*xt_hat[j];
		cv::Mat xt_tld=atxt_temp+btut;
		std::cout<<"xt_tld:"<<xt_tld<<"\n";
		std::cout<<"3.11\n";
		cv::Mat atsgm_temp=at*sgm[j]*at.t();
		cv::Mat sgm_tld=atsgm_temp+rt;
		std::cout<<"sgm_tld:"<<sgm_tld<<"\n";
		std::cout<<"3.12\n";	
		std::cout<<"qt:"<<qt<<"\n";
		cv::Mat sgmqt_temp=(sgm_tld+qt);
		cv::Mat sgmqt_inv_temp=sgmqt_temp.inv();
//		cv::Mat sgmtld_temp=sgm_tld*sgmqt_temp;
		cv::Mat kt=sgm_tld*sgmqt_inv_temp;
		std::cout<<"kt:"<<kt<<"\n";
		std::cout<<"3.14\n";
		std::cout<<"zt("<<(newpoints[j].x-width/2)*nz[j]/f<<","<<-(newpoints[j].y-height/2)*nz[j]/f<<","<<nz[j]<<"\n";
		double zt_data[3][1]={	{(newpoints[j].x-width/2)*nz[j]/f},
								{-(newpoints[j].y-height/2)*nz[j]/f},{nz[j]} };
		cv::Mat zt(3,1,CV_64FC1,zt_data);

		std::cout<<"zt:"<<zt<<"\n";
		std::cout<<"xt_hat[j]:"<<xt_hat[j]<<"\n";
		std::cout<<"kt:"<<kt<<"\n";
		std::cout<<"kt*(zt-xt_hat[j]):"<<kt*(zt-xt_hat[j])<<"\n";
		cv::Mat xt_temp=xt_hat[j];
		cv::Mat zt_temp=(zt-xt_hat[j]);
		cv::Mat kt_temp=kt*zt_temp;
		xt_hat[j]=xt_temp+kt_temp;
//		xt_hat[j]=xt_temp+kt*(zt-xt_hat[j]);
//		xt_hat[j]=xt_hat[j]+kt*(zt-xt_hat[j]);
		std::cout<<"xt_hat[j]:"<<xt_hat[j]<<"\n";
		std::cout<<"3.2\n";
		double I_data[3][3]={ {1,0,0},{0,1,0},{0,0,1} };
		cv::Mat I(3,3,CV_64FC1,I_data);
		std::cout<<"I:"<<I<<"\n";
		std::cout<<"(I-kt)*sgm_tld:"<<(I-kt)*sgm_tld<<"\n";
		cv::Mat sgm_temp=(I-kt);
		std::cout<<"(I-kt)"<<(I-kt)<<"\n";
		sgm[j]=sgm_temp*sgm_tld;

//		sgm[j]=(I-kt)*sgm_tld;
		std::cout<<"4\n";
	}

	
	
	for(int j=0;j<points.size();j++){
//----矢印描写---
//		float L1=std::abs(newpoints[j].x-jnewpoints[j].x);//sqrt(z[j]);
//		float L1=std::abs(newpoints[j].x-x_hat[j].at<double>(0,0));//sqrt(z[j]);
		float L2=sqrt((newpoints[j].x-jnewpoints[j].x)*(newpoints[j].x-jnewpoints[j].x)
			+(newpoints[j].y-jnewpoints[j].y)*(newpoints[j].y-jnewpoints[j].y));//sqrt(z[j]);
//newpoints==points+LK
//jnewpoints==points+jacobi
//newpoints-jnewpoints==LK-jacobi
//2*points-jnewpoint==points-jacobi
//		std::cout<<"newpoints[j]:"<<newpoints[j]<<"\n";
//		std::cout<<"x_hat[j]:"<<x_hat[j]<<"\n";
		ImageProcesser::cvArrow(&Limg_view,
				cv::Point((int)points[j].x,
					(int)points[j].y),
				cv::Point((int)(newpoints[j].x-jnewpoints[j].x+points[j].x),
					(int)(newpoints[j].y-jnewpoints[j].y+points[j].y)),
				cv::Scalar(0,200,200));//
		ImageProcesser::cvArrow(&Limg_view,
			cv::Point((int)points[j].x,
				(int)points[j].y),
/*			cv::Point((int)(f*xt_hat[j].at<double>(0,0)/xt_hat[j].at<double>(2,0)+width/2),
				(int)(f*xt_hat[j].at<double>(1,0)/xt_hat[j].at<double>(2,0)+height)),
*/	
			cv::Point((int)(f*xt_hat[j].at<double>(0,0)/nz[j]+width/2),
				(int)(f*xt_hat[j].at<double>(1,0)/nz[j]+height)),
			cv::Scalar(200,0,200));//
	
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

