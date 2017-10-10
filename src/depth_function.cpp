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
//set depth cvbridge image
	void ImageProcesser::setdepth(void){
	  depth_queue.callOne(ros::WallDuration(100));
	}
//set depth mat image
	void ImageProcesser::setdepth_img(void){
		depth_img=depthimg->image.clone();
	}
	//publish depth image
	void ImageProcesser::pub_depthimg(void){
		pub_dpt.publish(depthimg->toImageMsg());
	}

