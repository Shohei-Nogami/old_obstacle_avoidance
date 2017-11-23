#include"img_prc_cls.h"

	void ImageProcesser::depth_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		try{
			depthimg= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
			msg->encoding.c_str());
			return ;
		}
	}
	void ImageProcesser::avedepth_callback(const obst_avoid::sqr_point3d::ConstPtr& msg)
	{
		sp3d.sqr_p3d= msg->sqr_p3d;	
	}
	//set depth関連を一括管理
	void ImageProcesser::set_depth(void){
		if(isdepth())
			setPrevdepth();
		setcvdepth();
		setmtdepth();
	}
	//wheater there is mtdepth
	bool ImageProcesser::isdepth(void){
		if(depth_img.empty())
			return false;
		else
			return true;
	}
	//set previous mat depth
	void ImageProcesser::setPrevdepth(void){
		 Predepth=depth_img.clone();
	}
	//set depth cvbridge image
	void ImageProcesser::setcvdepth(void){
		depth_queue.callOne(ros::WallDuration(1));
	}
	//set depth mat image
	void ImageProcesser::setmtdepth(void){
		depth_img=depthimg->image.clone();
	}
	//publish depth image
	void ImageProcesser::pub_depthimg(void){
		pub_dpt.publish(depthimg->toImageMsg());
	}
	void ImageProcesser::setave3d(void){
		avedepth_queue.callOne(ros::WallDuration(1));
	}

	//Linear approximation
	void ImageProcesser::approx_depth_img(void){	
	
		for(int i=0;i<height;i++){
			for(int j=0;j<width;j++){
				if(std::isnan(depth_img.at<float>(i,j))){
					depth_img.at<float>(i,j)=0;
					continue;
				}
				if(std::isinf(depth_img.at<float>(i,j))){
					depth_img.at<float>(i,j)=0;
				}
			}
		}
	//if(depth_img<flost>(i,j)==0)
	//	if(std::isnan(depth_img.at<float>(i,j))||std::isinf(depth_img.at<float>(i,j)))
	//if(depth_img<flost>(i,j)!=0)
	//	if(!std::isnan(depth_img.at<float>(i,j))&&!std::isinf(depth_img.at<float>(i,j)))
	
		float depth,depth1,depth2;
		for(int i=0;i<height;i++){
			depth=0;
			depth1=0;
			depth2=0;
			for(int j=0,count=0;j<width-1;j++){
				depth=depth_img.at<float>(i,j);
				//|val|nan|のとき
				if(depth!=0 && depth_img.at<float>(i,j+1)==0){
					depth1=depth;
					count++;
				}

				else if(depth==0){
					//|nan|nan|の区間
					if(depth_img.at<float>(i,j+1)==0)
						count++;
					//|nan|val|のとき
					else{
						depth2=depth_img.at<float>(i,j+1);
						//左端がnanのとき
						if(depth1==0){
								for(int k=0;k<count+1;k++)
									depth_img.at<float>(i,j-k)=depth2;
						
						}
						else{
											
							for(int k=0;k<count;k++)
								depth_img.at<float>(i,j-k)=depth2-(depth2-depth1)/(count+1)*(k+1);
						}
					}
				}
				//右端がnanのとき
				if(j==(width-1)-1 &&depth_img.at<float>(i,j+1)==0){
					for(int k=0;k<count;k++)
						depth_img.at<float>(i,j+1-k)=depth1;
				}
			}
		}
	}
