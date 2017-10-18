#include"img_prc_cls.h"

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
//---特徴点(keypoints)を得る-------------
//	auto detector = cv::ORB(max_points, 1.25f, 4, 7, 0, 2, 0, 7);
//	detector.detect(PreLgray, keypoints);

//---keypointsをpointsにコピー-----------
/*	float ptz;
	for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
		 itk != keypoints.end(); ++itk){
		ptz=depth_img.at<float>(
				itk->pt.y,
				itk->pt.x
				);
		if(!std::isnan(ptz)){
			pts.push_back(itk->pt);
			pz.push_back(ptz);
		}
	}

	if(!pts.size()){
		return ;
	}*/
/*	if(judge_feature_points()){
		auto detector = cv::ORB(max_points, 1.25f, 4, 7, 0, 2, 0, 7);
		detector.detect(PreLgray, keypoints);
		add_feature_points();
	}
*/
	for(int i=0;i<cn;i++){
		for(int j=0;j<cn;j++){
			clp_img[i][j]=PreLgray(cv::Rect((int)(j*(width)/cn),(int)(i*(height)/cn),(int)(width/cn),(int)(height/cn)));
		}
	}
	count_feature_points();
//	auto detector = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
//	detector = cv::ORB(clp_max_points, 1.25f, 4, 7, 0, 2, 0, 7);
	add_feature_points();
	if(!pts.size()){
		return ;
	}
//	std::cout<<"points size:"<<pts.size()<<"\n";
//---オプティカルフローを得る-----------------------------
	cv::calcOpticalFlowPyrLK(PreLgray,Lgray, pts, npts, sts, ers, cv::Size(21,21), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 0);
//Delete the point that not be matched
	float pnz;
	for(int i=0,k=0;i<pts.size();i++){
		
		if(sts[i]&&!std::isnan(depth_img.at<float>(npts[i].y,npts[i].x))){
			points.push_back(pts[i]);
			newpoints.push_back(npts[i]);
			z.push_back(pz[i]);
			nz.push_back(depth_img.at<float>(npts[i].y,npts[i].x));
		}
	}
//memory release
//	PreLgray.release();
//	Lgray.release();
//-----画像ヤコビアンを用いて--------------------
//-----ロボットの移動によるnewpointsの求める-----
	double d=0.276;//車輪幅
	double v=global_dx;//visual odometry z座標
	double w=(vr-vl)/d;//回転角速度
	double sh=w*dt;//回転角
	double dx=global_dy;//visual odometry x座標
	w=dyaw;
	for(int j=0;j<points.size();j++){
		::obst_avoid::points point;
		float X=(float)(points[j].x-width/2.0);//-width;
		float Y=(float)(points[j].y-height/2.0);//-height;
		float value_x;
		float value_y;
		value_x=(float)((
		  	dx/z[j]-X/z[j]*v
		  	-(1+pow(X,2.0)/f)*w
		  	));
		value_y=(float)(
			  	-(Y/z[j]*v)
			  	-(X*Y*w/f
			  	));
	
//----矢印描写---
/*		float opticalflow_size_prev=sqrt(
			std::pow((newpoints[j].x-points[j].x)
					,2.0)+
			std::pow((newpoints[j].y-points[j].y)
					,2.0));
		float opticalflow_size=sqrt(
			std::pow((newpoints[j].x-points[j].x+value_x)
					,2.0)+
			std::pow((newpoints[j].y-points[j].y+value_y)
					,2.0));
*/
		float L1=(std::abs(newpoints[j].x-points[j].x+value_x)
				+std::abs(newpoints[j].y-points[j].y+value_y))*z[j];

/*		if(std::isnan(prvp[j]))
			prvp[j]=1;
		curp.push_back(prvp[j]*
			(1/(sqrt(2*PI)*sig*sig)*exp(-(L1/(2*sig*sig)) ) ) );
		std::cout<<"current P"<<j<<":"<<curp[j]<<"\n";
//opticaleflowが一定サイズ以下のとき
		if(curp[j]>0.20){
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x),
					(+(int)points[j].y)),
				cv::Point(((int)newpoints[j].x+(int)value_x),
					(+(int)newpoints[j].y)+(int)value_y),
				cv::Scalar(255,255,255));//白
		}
		else{
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x),
					((int)points[j].y)),
				cv::Point(((int)newpoints[j].x),
					((int)newpoints[j].y)),
				cv::Scalar(0,200,200));//黄
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x
					),
					((int)points[j].y)),
				cv::Point(((int)points[j].x+(int)value_x),
					((int)points[j].y+(int)value_y)),
				cv::Scalar(200,0,200));//紫
		}*/
		double th_optt=th_opt;
		if(std::abs(dyaw)>0.01)
			th_optt=th_opt*(dyaw/0.01);
		if(L1<th_optt){
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x),
					(+(int)points[j].y)),
				cv::Point(((int)newpoints[j].x+(int)value_x),
					(+(int)newpoints[j].y)+(int)value_y),
				cv::Scalar(255,255,255));//白
		}
		else{
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x),
					((int)points[j].y)),
				cv::Point(((int)newpoints[j].x),
					((int)newpoints[j].y)),
				cv::Scalar(0,200,200));//黄
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x
					),
					((int)points[j].y)),
				cv::Point(((int)points[j].x+(int)value_x),
					((int)points[j].y+(int)value_y)),
				cv::Scalar(200,0,200));//紫
		}

/*		if(opticalflow_size<th_opt){
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x),
					(+(int)points[j].y)),
				cv::Point(((int)newpoints[j].x+(int)value_x),
					(+(int)newpoints[j].y)+(int)value_y),
				cv::Scalar(255,255,255));//白
		}
		else{
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x),
					((int)points[j].y)),
				cv::Point(((int)newpoints[j].x),
					((int)newpoints[j].y)),
				cv::Scalar(0,200,200));//黄
			ImageProcesser::cvArrow(&Limg_view,
				cv::Point(((int)points[j].x
					),
					((int)points[j].y)),
				cv::Point(((int)points[j].x+(int)value_x),
					((int)points[j].y+(int)value_y)),
				cv::Scalar(200,0,200));//紫
		}
*/
//output file
		std::ofstream ofss("./Documents/output_opticalflow.csv",std::ios::app);
		ofss<<points[j].x-width/2<<","//X
			<<points[j].y-height/2<<","//Y
			<<z[j]<<","//z
			<<dx<<","//dx
			<<v<<","//dz
			<<w<<","//dw
			<<dt<<","//dt
			<<","
			<<newpoints[j].x-points[j].x<<","//観測x
			<<newpoints[j].y-points[j].y<<","//観測y
			<<(double)value_x<<","//ヤコビx
			<<(double)value_y<<","//ヤコビy
			<<","
			<<newpoints[j].x-points[j].x-value_x<<","//観測x-jacobi
			<<newpoints[j].y-points[j].y-value_y<<","//観測y-jacobi
			<<std::endl;

	}
//publish用のcvbridge
//	cv_bridge::CvImagePtr PubLimg(new cv_bridge::CvImage);
//	PubLimg->encoding=sensor_msgs::image_encodings::BGR8;
//	PubLimg->image=Limg_view.clone();
//	pub_Limg.publish(PubLimg->toImageMsg());

//	ROS_INFO("process end");//debug

}
