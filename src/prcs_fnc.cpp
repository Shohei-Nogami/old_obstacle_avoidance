#include"image_process_zed_function.h"

//画像フレーム取得後呼び出される関数
void ImageProcesser::imageProcess()
{
	//debug
    std::cout<<"4:process\n";

	ros::Time process_start_time= ros::Time::now();
	ros::Duration process_time = ros::Time::now()-start_time;
//Move	
		double Move_nX=0;
		double Move_nY=0;
		int count=0;
		cv::Mat depth_img;
		cv::Mat Limg;
		Limg=org_img->image.clone();
		cv::Mat Limg_view=Limg.clone();//imshow用のMat
		if(PreLimg.empty()){	//差分を取るための処理(ループ１回目のみ処理をしない)
		}										
		else{
		//	ROS_INFO_STREAM("Processing\n");
//グレースケール化
			cv::Mat Lgray,PreLgray;
			cv::cvtColor(Limg,Lgray,CV_BGR2GRAY);
			cv::cvtColor(PreLimg,PreLgray,CV_BGR2GRAY);
			
//差分をとり２値化
			cv::Mat DifLgray,MaskLimg;
			cv::absdiff(Lgray,PreLgray,DifLgray);
//差分diffのうち、閾値thを超えている部分を1、それ以外を0としてmaskに出力
   	 		cv::threshold(DifLgray,MaskLimg,10,1,cv::THRESH_BINARY);
//マスクmaskのうち、1(True)の部分を白(0)に、0(False)の部分を黒(255)にしてim_maskに出力
       		cv::threshold(MaskLimg,MaskLimg,0,255,cv::THRESH_BINARY);				
//膨張縮小処理
				
			cv::erode(MaskLimg,MaskLimg,cv::Mat(),cv::Point(-1,-1), 1);
			cv::dilate(MaskLimg,MaskLimg,cv::Mat(),cv::Point(-1,-1), 4);

//表示用maskLimg
			cv::Mat MaskLimg_view=MaskLimg.clone();
//memory release
			DifLgray.release();
			
//輪郭抽出(左カメラ)
			cv::Mat act;
			double sum_area=0;//差分合計面積
			act=MaskLimg.clone();
			std::vector<std::vector<cv::Point> >  contours;
	       	cv::findContours(act, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			
			std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
			std::vector<cv::Rect> boundRect( contours.size() );//
			
//差分画像を矩形で囲む(左上、右下の座標を取る)
			int rect_num=0;//矩形の数
			for( int i = 0; i < contours.size(); i++ )
			{
				double area=cv::contourArea(contours.at(i));
				sum_area+=area;
				if(area>200){//面積が一定以上であれば//改善の余地あり
				cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true );
				boundRect[rect_num++] = cv::boundingRect(cv::Mat(contours_poly[i]) );//矩形の作成、格納

				}
			}
			act.release();//memory release
				
//-----オプティカルフローをかける矩形
			cv::Mat opt_PreLimg[rect_num];
			cv::Mat opt_Limg[rect_num]; 
//-------------------矩形の重ね合わせ---------------------

			int rect_num0=rect_num;
			int RectStatus[rect_num0];
			cv::Point2i RectPoint_tl[rect_num0];
			cv::Point2i RectPoint_br[rect_num0];
			int rect_size=0;
				
			for(int i=0;i<rect_num0;i++){
				RectStatus[i]=0;//初期化
			}
			for(int i=0;i<rect_num0;i++){
				if(RectStatus[i]==0){
					for(int j=0;j<rect_num0;j++){
						if(rect_size>2){	
							for(int k=0;k<rect_size-1;k++){
								if( ( RectPoint_tl[k].x < boundRect[j].br().x ) &&
									( boundRect[j].tl().x < RectPoint_br[k].x ) &&
									( RectPoint_tl[k].y < boundRect[j].br().y ) &&
									( boundRect[j].tl().y < RectPoint_br[k].y ) ) {		
									
									if(RectPoint_tl[k].x > RectPoint_tl[k+1].x){//左上x
											RectPoint_tl[k].x=RectPoint_tl[k+1].x;
									}
									if(RectPoint_tl[k].y > RectPoint_tl[k+1].y){//左上y
										RectPoint_tl[k].y=RectPoint_tl[k+1].y;
									}	
									if(RectPoint_br[k].x < RectPoint_br[k+1].x){//右下x
										RectPoint_br[k].x=RectPoint_br[k+1].x;
									}
									if(RectPoint_br[k].y < RectPoint_br[k+1].y){//右下y
										RectPoint_br[k].y=RectPoint_br[k+1].y;
									}
											
											
								}
							}
							rect_size--;
						}
						if(RectStatus[j]==0){//重ねたあとの矩形は処理しない
							for(int k=0;k<rect_size;k++){	
								if( ( RectPoint_tl[k].x < boundRect[j].br().x ) &&
									( boundRect[j].tl().x < RectPoint_br[k].x ) &&
									( RectPoint_tl[k].y < boundRect[j].br().y ) &&
									( boundRect[j].tl().y < RectPoint_br[k].y ) ) {								
									if(RectPoint_tl[k].x > boundRect[j].tl().x){//左上x
										RectPoint_tl[k].x=boundRect[j].tl().x;
									}
									if(RectPoint_tl[k].y > boundRect[j].tl().y){//左上y
										RectPoint_tl[k].y=boundRect[j].tl().y;
									}	
									if(RectPoint_br[k].x < boundRect[j].br().x){//右下x
										RectPoint_br[k].x=boundRect[j].br().x;
									}
									if(RectPoint_br[k].y < boundRect[j].br().y){//右下y
										RectPoint_br[k].y=boundRect[j].br().y;
									}
									RectStatus[j]=1;
								}								
							}
						}			
							
						if(RectStatus[j]==0&&RectStatus[i]==0&&i!=j){//重ねたあとの矩形は処理しない
							if( ( boundRect[i].tl().x < boundRect[j].br().x ) &&
								( boundRect[j].tl().x < boundRect[i].br().x ) &&
								( boundRect[i].tl().y < boundRect[j].br().y ) &&
								( boundRect[j].tl().y < boundRect[i].br().y )  ) {
							//当たっている場合にこの部分が実行されます
								if(boundRect[i].tl().x > boundRect[j].tl().x){//左上x
									RectPoint_tl[rect_size].x=boundRect[j].tl().x;
								}
								else{
									RectPoint_tl[rect_size].x=boundRect[i].tl().x;
								}
								if(boundRect[i].tl().y > boundRect[j].tl().y){//左上y
									RectPoint_tl[rect_size].y=boundRect[j].tl().y;
								}
								else{
									RectPoint_tl[rect_size].y=boundRect[i].tl().y;
								}	
								if(boundRect[i].br().x < boundRect[j].br().x){//右下x
									RectPoint_br[rect_size].x=boundRect[j].br().x;
								}
								else{	
									RectPoint_br[rect_size].x=boundRect[i].br().x;
								}
								if(boundRect[i].br().y < boundRect[j].br().y){//右下y
									RectPoint_br[rect_size].y=boundRect[j].br().y;
								}
								else{
									RectPoint_br[rect_size].y=boundRect[i].br().y;
								}
								RectStatus[i]=1;
								RectStatus[j]=1;
								rect_size++;	
								
								
							}
						}	
					}
				}
			}		
//合成した矩形と矩形の配列をつなげる	
			for(int j=0,i=rect_size;j<rect_num;j++){
				if(RectStatus[j]==0){
					RectPoint_tl[i].x=boundRect[j].tl().x;
					RectPoint_tl[i].y=boundRect[j].tl().y;
					RectPoint_br[i].x=boundRect[j].br().x;
					RectPoint_br[i].y=boundRect[j].br().y;
					rect_size++;
					i++;
				}
			}
//矩形の合成終了

//-----オプティカルフロー関数用パラメータ-----
			std::vector<cv::Point2f> points[rect_size];	//特徴点
			std::vector<cv::Point2f> newpoints[rect_size];	//移動後の特徴点
			std::vector<uchar> status[rect_size];	
			std::vector<float> errors;
//-----特徴点抽出用変数-----		
			std::vector<cv::KeyPoint> keypoints;

			for(int i=0;i<rect_size;i++)
//----------矩形それぞれに対する処理----------
			{
//合成矩形で囲む
				cv::rectangle(Limg_view,RectPoint_tl[i],RectPoint_br[i],cv::Scalar(0,200,0),3,4);
//現フレームの動体と予測された範囲を矩形として取り出す
				opt_Limg[i]=Lgray(cv::Rect(RectPoint_tl[i],RectPoint_br[i]));
//前フレームの動体と予測された範囲を矩形として取り出す
				opt_PreLimg[i]=PreLgray(cv::Rect(RectPoint_tl[i],RectPoint_br[i]));
			
//参照URL:http://opencv.jp/opencv-2svn/cpp/motion_analysis_and_object_tracking.html#cv-calcopticalflowpyrlk

 
//---特徴点(keypoints)を得る-------------

				auto detector = cv::ORB(3000, 1.25f, 4, 7, 0, 2, 0, 7);
				detector.detect(opt_PreLimg[i], keypoints);

//debug----------------------------------
				std::vector<cv::Point2f> debug_points;
				for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
					itk != keypoints.end(); ++itk){
						debug_points.push_back(itk->pt);
				}
				int isz_num=0;
				for(int y=0;y<height;y++){
					for(int x=0;x<width;x++){
						float isz=depth_img.at<float>(
                            y,
                            x
                            );
						if(std::isnan(isz))
							isz_num++;
					}
				}
//--------------------------------
//---keypointsをpointsにコピー-----------
				for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
					 itk != keypoints.end(); ++itk){
                    float z=depth_img.at<float>(
                            RectPoint_tl[i].y+itk->pt.y,
                            RectPoint_tl[i].x+itk->pt.x
                            );	
					if(!std::isnan(z)){
//						std::cout<<"push back point\n";
						points[i].push_back(itk->pt);
							
					}
//zまわりの8点を探索し、zのばらつきが閾値以下かつ8点すべてが!nanではない時
//その平均値を測定可能とする
					else{
						float around_z;
						int n=0;
						int is_around_z=0;
						int num_around_z=0;
						float sum_around_z=0;
						float min_around_z=100,max_around_z=0;
						for(int k=-1;k<=1;k++){
							for(int l=-1;l<=1;l++){
								if(k==0&&l==0)
									continue;
								around_z=depth_img.at<float>(
									RectPoint_tl[i].y+itk->pt.y+k,
									RectPoint_tl[i].x+itk->pt.x+l
								);
								if(std::isnan(around_z))
									is_around_z++;
								else{
									if(min_around_z>around_z)
										min_around_z=around_z;
									if(max_around_z>around_z)
										max_around_z=around_z;
									num_around_z++;
									sum_around_z+=around_z;
								}
							}
						}

						if(is_around_z<=1&&(max_around_z-min_around_z)<0.05){
							points[i].push_back(itk->pt);
						}
					}	
				}	

				if(!points[i].size()){
					continue;
				}
//---オプティカルフローを得る-----------------------------
				cv::calcOpticalFlowPyrLK(opt_PreLimg[i],opt_Limg[i], points[i], newpoints[i], status[i], errors, cv::Size(/*21,21*/15,15), 3,cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 0);
				
			}//それぞれの矩形に対する処理(for文)終了
			
//memory release			
			PreLgray.release();
			Lgray.release();
//-----画像ヤコビアンを用いて--------------------
//-----ロボットの移動によるnewpointsの求める-----			
			double d=0.276;//0.3;
			double v=visual_dx;//visual odometry
			double w=(vr-vl)/(d);
			double sh=w*dt;
			double dx=visual_dy;//visual odometry		
			dt=(new_time-prev_time);
			float delta_cx=cx-width/2.0;
			float delta_cy=cy-height/2.0;
			std::vector<cv::Point2f>  theory_newpoints[rect_size];
			w=dyaw;
//msg for publisher
			::obst_avoid::moving_pointsArray movepointsArray;
			movepointsArray.rect_size=rect_size;

            for(int i=0;i<rect_size;i++){
				::obst_avoid::moving_points movepoints;
				movepoints.rect_tl.x=RectPoint_tl[i].x;
				movepoints.rect_tl.y=RectPoint_tl[i].y;
				movepoints.rect_br.x=RectPoint_br[i].x;
				movepoints.rect_br.y=RectPoint_br[i].y;
				movepoints.point_size=points[i].size();
                for(int j=0;j<points[i].size();j++){
					::obst_avoid::points point;
					
					float z;
					z=depth_img.at<float>(
							RectPoint_tl[i].y+(int)points[i][j].y,
							RectPoint_tl[i].x+(int)points[i][j].x
							);
//					float X=((float)f/1000.0)*(float)(RectPoint_tl[i].x+points[i][j].x+delta_cx-cx);//-width;
//					float Y=((float)f/1000.0)*(float)(RectPoint_tl[i].y+points[i][j].y+delta_cy-cy);//-height;
					float X=(float)(RectPoint_tl[i].x+points[i][j].x+delta_cx-cx)/f;//-width;
					float Y=(float)(RectPoint_tl[i].y+points[i][j].y+delta_cy-cy)/f;//-height;
					float value_x=(float)((
								dx/dt/z-X/z*v/dt
								-(1+pow(X,2.0))*w/dt
								)*dt);
					float value_y=(float)((
								-(Y/z*v/dt)
								-(X*Y*w/dt
								))*dt);
					if(!std::isnan(z)){
	//					std::cout<<"value_x:"<<value_x<<'\n';
	//					std::cout<<"value_y:"<<value_y<<'\n';
					}
					else{
						float around_z;
						int n=0;
						int is_around_z=0;
						int num_around_z=0;
						float min_around_z=100,max_around_z=0;
						float sum_around_z=0;
						for(int k=-1;k<=1;k++){
							for(int l=-1;l<=1;l++){
								if(k==0&&l==0)
									continue;
								around_z=depth_img.at<float>(
									RectPoint_tl[i].y+(int)points[i][j].y+k,
									RectPoint_tl[i].x+(int)points[i][j].x+l
								);
								if(std::isnan(around_z))
									is_around_z++;
								else{
									num_around_z++;
									sum_around_z+=around_z;
									if(min_around_z>around_z)
										min_around_z=around_z;
									if(max_around_z<around_z)
										max_around_z=around_z;
								}
							}
						}
						if(is_around_z<=1&&(max_around_z-min_around_z)<0.05){
							z=sum_around_z/num_around_z;
						}
					}	

//----矢印描写----
					if(!std::isnan(z)){
						float eval;
						eval=sqrt(
							std::pow((newpoints[i][j].x-points[i][j].x
									+value_x)//-theory_newpoints[i][j].x)
									,2.0)+	
							std::pow((newpoints[i][j].y-points[i][j].y
									+value_y)//-theory_newpoints[i][j].y)
									,2.0));	
						

						float opticalflow_size_prev=sqrt(
							std::pow((newpoints[i][j].x-points[i][j].x)
									,2.0)+	
							std::pow((newpoints[i][j].y-points[i][j].y)
									,2.0));	
						
						float opticalflow_size=sqrt(
							std::pow((newpoints[i][j].x-points[i][j].x-value_x)
									,2.0)+	
							std::pow((newpoints[i][j].y-points[i][j].y-value_y)
									,2.0));	

//
						float th_val=1.0;
						if((value_x>th_val||value_y>th_val||value_x<-th_val||value_y<-th_val)&&
							opticalflow_size>0&&
							opticalflow_size<5){
							ImageProcesser::cvArrow(&Limg_view,
								cv::Point((RectPoint_tl[i].x+(int)points[i][j].x
									),
									(RectPoint_tl[i].y+(int)points[i][j].y)),
								cv::Point((RectPoint_tl[i].x+(int)newpoints[i][j].x
									+(int)value_x),//theory_newpoints[i][j].x),
									(RectPoint_tl[i].y+(int)newpoints[i][j].y)
									+(int)value_y),//-(int)theory_newpoints[i][j].y),
								cv::Scalar(255,255,255));	
						}
						else{
							ImageProcesser::cvArrow(&Limg_view,
								cv::Point((RectPoint_tl[i].x+(int)points[i][j].x
									),
									(RectPoint_tl[i].y+(int)points[i][j].y)),
								cv::Point((RectPoint_tl[i].x+(int)points[i][j].x-(int)value_x),//theory_newpoints[i][j].x),
									(RectPoint_tl[i].y+(int)points[i][j].y-(int)value_y)),//-(int)theory_newpoints[i][j].y),
								cv::Scalar(200,0,200));	

							ImageProcesser::cvArrow(&Limg_view,
								cv::Point((RectPoint_tl[i].x+(int)points[i][j].x
									),
									(RectPoint_tl[i].y+(int)points[i][j].y)),
								cv::Point((RectPoint_tl[i].x+(int)newpoints[i][j].x
									),//-(int)value_x),//theory_newpoints[i][j].x),
									(RectPoint_tl[i].y+(int)newpoints[i][j].y)
									),//-(int)value_y),//-(int)theory_newpoints[i][j].y),
								cv::Scalar(0,200,200));	


							point.x=newpoints[i][j].x;
							point.y=newpoints[i][j].y;
							movepoints.points.push_back(point);
							
						}
					}
					movepointsArray.moving_pointsArray.push_back(movepoints);
                }
            }
		
		cv_bridge::CvImagePtr PubLmsk(new cv_bridge::CvImage);
		PubLmsk->encoding=sensor_msgs::image_encodings::MONO8;
		PubLmsk->image=MaskLimg_view.clone();
		pub_Lmsk.publish(PubLmsk->toImageMsg());
		}//else文終了					
//１つ前の画像処理時間を取得
//現フレームを1つ前のフレームとする
		Limg.copyTo(PreLimg);
		Limg.release();
//publish用のcvbridge			
		cv_bridge::CvImagePtr PubLimg(new cv_bridge::CvImage);
		PubLimg->encoding=sensor_msgs::image_encodings::BGR8;
		PubLimg->image=Limg_view.clone();
		pub_Limg.publish(PubLimg->toImageMsg());
//--------------------fps表示--------
	ros::Duration delta_t_0 = ros::Time::now() - process_start_time;
	double delta_t_sec0 = delta_t_0.toSec();
	double fps=1/delta_t_sec0;
//--------------------------------------
}

