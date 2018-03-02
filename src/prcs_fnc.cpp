#include"img_prc_cls.h"

bool wf_f=false;
bool wfo_f=false;//true;//
int wfo_c=0;
const int wfo_cmax=10;
bool wfo_cf=true;
const int th_dis_bias=2;
const int th_count=5;
//画像フレーム取得後呼び出される関数
void ImageProcesser::imageProcess(){
	Limg_view=Limg.clone();//imshow用のMat
	if(!is_Previmage())
		return ;

//グレースケール化
	cv::cvtColor(Limg,Lgray,CV_BGR2GRAY);
	cv::cvtColor(PreLimg,PreLgray,CV_BGR2GRAY);
//separate original image
	for(int i=0;i<cnh;i++){
		for(int j=0;j<cnw;j++){
			clp_img[i][j]=PreLgray(cv::Rect((int)(j*(width)/cnw),(int)(i*(height)/cnh),(int)(width/cnw),(int)(height/cnh)));
		}
	}
//tracking and adding feature points
	count_feature_points();
	add_feature_points();
	if(!pts.size()){
		return ;
	}
//修正必要
	float X,Y;
	cv::Point2f ppt;

	for(int j=0;j<pts.size();j++){
		X=(float)(pts[j].x-width/2.0);
		Y=(float)(pts[j].y-height/2.0);
		float value_x;
		float value_y;
/*
//only visual odometry
		ppt.x=pts[j].x- (float)(
			dx/pz[j]-X/pz[j]*dz
			-(f+pow(X,2.0)/f)*dyaw
			);
			ppt.y=pts[j].y-(float)(
					-(Y/pz[j]*dz)
					-(X*Y*dyaw/f
					));
*/

//v by wheel odometry and dyaw by visual odometry
		ppt.x=pts[j].x- (float)(
			w_v*sin(-dyaw)*dt/pz[j]-X/pz[j]*w_v*cos(-dyaw)*dt
			-(f+pow(X,2.0)/f)*dyaw
			);

		ppt.y=pts[j].y-(float)(
				-(Y/pz[j]*w_v*cos(-dyaw)*dt)
				-(X*Y*dyaw/f
				));

		jnpts.push_back(ppt) ;
	}
/*//publish wheel vel data
	obst_avoid::dvw dvw_msg;	
	dvw_msg.v=w_v;
	dvw_msg.w=w_w;
	dvw_msg.dv=dz/dt-dvw_msg.v;//*dt;
	dvw_msg.dw=dyaw/dt-dvw_msg.w;//*dt;
	pub_dvw.publish(dvw_msg);
*/
//set predictional points to use LK method	
	npts.insert(npts.end(),jnpts.begin(),jnpts.end());
//---オプティカルフローを得る-----------------------------
	cv::calcOpticalFlowPyrLK(PreLgray,Lgray, pts, npts, sts, ers, cv::Size(13,13), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);

//renew points, depth, tracking count and (static and movement) flag
	float pnz;
	for(int i=0;i<pts.size();i++){
		float depth_np=depth_img.at<float>(npts[i].y,npts[i].x);
		if(sts[i]&&!std::isnan(depth_np)&&!std::isinf(depth_np)&&depth_np>=0.5){
			points.push_back(pts[i]);
			newpoints.push_back(npts[i]);
			z.push_back(pz[i]);
			jnewpoints.push_back(jnpts[i]);
			nz.push_back(depth_np);
			tracking_count.push_back(tracking_count_p[i]+1);
			is_stc_pts.push_back(is_stc_pts_p[i]);
			is_mv_pts.push_back(is_mv_pts_p[i]);
		}
	}


	for(int j=0;j<points.size();j++){
//----矢印描写---
		float L1=std::abs(newpoints[j].x-jnewpoints[j].x);//sqrt(z[j]);
		float L2=sqrt((newpoints[j].x-jnewpoints[j].x)*(newpoints[j].x-jnewpoints[j].x)
			+(newpoints[j].y-jnewpoints[j].y)*(newpoints[j].y-jnewpoints[j].y));
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

