#include"img_prc_cls.h"

//callback function
	void ImageProcesser::image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
	    try{
		org_img[lpf_count++]= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//		prv_img[lpf_count++]=org_img->image.clone();
//		std::cout<<"0\n";
		if(lpf_count%lpf_value==0)
			lpf_count=0;
	    }
	    catch(cv_bridge::Exception& e) {
		    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
		    msg->encoding.c_str());
		    return ;
	    }
	}
//set original image	要改善(1つ前の画像の有無等)
	void ImageProcesser::setimage(void){
		if(isLimage())
			setPrevimage();
		set_orgimg();
//		if(!prv_img[lpf_value-1].empty())
			set_Limg();
	}
	//set original image
	void ImageProcesser::set_orgimg(void){
		for(int i=0;i<lpf_value;i++)
			image_queue.callOne(ros::WallDuration(1));
	}
	//set Left image
	void ImageProcesser::set_Limg(void){
//						
		org_img[0]->image.convertTo(lpf_img[0],CV_32FC3);
		sum_img=lpf_img[0];
		std::cout<<"1\n";
		for(int i=1;i<lpf_value;i++){
			org_img[i]->image.convertTo(lpf_img[i],CV_32FC3);
			sum_img=sum_img+lpf_img[i];
		}
		std::cout<<"1\n";
		ave_img=sum_img/lpf_value;
		std::cout<<"1\n";
		ave_img.convertTo(Limg, CV_8UC3 );		

//
//		Limg=org_img->image.clone();
	}
	//set previous image
	void ImageProcesser::setPrevimage(void){
//		PreLimg=org_img->image.clone();
		PreLimg=Limg.clone();
	}
//wheater image exist
	bool ImageProcesser::isPrevimage(void){
		if(PreLimg.empty())
			return false;
		else
			return true;
	}
	bool ImageProcesser::isLimage(void){
		if(Limg.empty())
			return false;
		else
			return true;
	}
	//----Publish image----
	//publish original image
	void ImageProcesser::pub_org_img(void){
//		pub_orgimg.publish(org_img->toImageMsg());
	}
	//publish view left image
	void ImageProcesser::pub_left_img(void){
		cv_bridge::CvImagePtr PubLimg(new cv_bridge::CvImage);
		PubLimg->encoding=sensor_msgs::image_encodings::BGR8;
		PubLimg->image=Limg_view.clone();
		pub_Limg.publish(PubLimg->toImageMsg());
	}

