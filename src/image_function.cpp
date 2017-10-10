#include"img_prc_cls.h"

//callback function
	void ImageProcesser::image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
	    try{
		org_img= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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
		set_Limg();
	}
	//set original image
	void ImageProcesser::set_orgimg(void){
		image_queue.callOne(ros::WallDuration(100));
	}
	//set Left image
	void ImageProcesser::set_Limg(void){
		Limg=org_img->image.clone();
	}
	//set previous image
	void ImageProcesser::setPrevimage(void){
		PreLimg=org_img->image.clone();
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
		pub_orgimg.publish(org_img->toImageMsg());
	}
	//publish view left image
	void ImageProcesser::pub_left_img(void){
		cv_bridge::CvImagePtr PubLimg(new cv_bridge::CvImage);
		PubLimg->encoding=sensor_msgs::image_encodings::BGR8;
		PubLimg->image=Limg_view.clone();
		pub_Limg.publish(PubLimg->toImageMsg());
	}

