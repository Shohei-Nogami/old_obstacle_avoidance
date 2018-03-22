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

	void ImageProcesser::filter_process(void){
		int FILTER_TYPE=MEDIAN_FILTER;//AVERAGE_FILTER;//NOTHING;
		if(FILTER_TYPE==MEDIAN_FILTER){
//----median filter
			//median filter parameter
			int median_param=5;
			//use opencv function
			if(use_cv_function){
				cv::medianBlur(depth_img,depth_img,ksize);
				for(int h=0;h<height/ksize;h++){
					for(int w=0;w<width/ksize;w++){
						depth_image.at<float>(h,w)=depth_img.at<float>(h*ksize,w*ksize);
					}
				}
			}//end if
			//without to use opencv function
			else{//self made
				for(int h=0;h<height/ksize;h++){
					for(int w=0;w<width/ksize;w++){
						for(int l=-median_param/2;l<=median_param/2;l++){
							for(int m=-median_param/2;m<=median_param/2;m++){
								if(std::isnan(depth_img.at<float>(h*ksize+l,w*ksize+m))
									||std::isinf(depth_img.at<float>(h*ksize+l,w*ksize+m)) 
									||h+l<0||height/ksize<=h+l
									||w+m<0||width/ksize<=w+m ){
									continue;
								}
								depth_median.push_back(depth_img.at<float>(h*ksize+l,w*ksize+m) );
							}
						}
						std::sort(depth_median.begin(),depth_median.end());
						if((int)depth_median.size()){
							depth_image.at<float>(h,w)=depth_median[(int)depth_median.size()/2];
						}
						else{
							depth_image.at<float>(h,w)=0;						
						}
						depth_median.clear();
					}
				}
			}//end else
		}
		else if(FILTER_TYPE==AVERAGE_FILTER){
			for(int h=0;h<height/ksize;h++){
				for(int w=0;w<width/ksize;w++){
					depth_sum=0;
					depth_element_num=0;
					for(int l=-ksize/2;l<=ksize/2;l++){
						for(int m=-ksize/2;m<=ksize/2;m++){
							if(std::isnan(depth_img.at<float>(h*ksize+l,w*ksize+m))
								||std::isinf(depth_img.at<float>(h*ksize+l,w*ksize+m))
								||h+l<0||height/ksize<=h+l
								||w+m<0||width/ksize<=w+m ){
								continue;
							}
							depth_sum+=depth_img.at<float>(h*ksize+l,w*ksize+m);
							depth_element_num++;
						}
					}
					depth_image.at<float>(h,w)=depth_sum/depth_element_num;						
				}
			}
		}
		else if(FILTER_TYPE==NOTHING){
			for(int h=0;h<height/ksize;h++){
				for(int w=0;w<width/ksize;w++){
					if(std::isnan(depth_img.at<float>(h*ksize,w*ksize))
						||std::isinf(depth_img.at<float>(h*ksize,w*ksize)) ){
						depth_image.at<float>(h,w)=0;						
					}
					else{
						depth_image.at<float>(h,w)=depth_img.at<float>(h*ksize,w*ksize);
					}
				}
			}
		}
	}
