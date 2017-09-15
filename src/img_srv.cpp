#include"image_process_zed_function.h"

bool img_srv(const sensor_msgs::ImageConstPtr& img)
{
	try{
		org_img=cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);

	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
		msg->encoding.c_str());
		return false;
	}
	return true;
}

int main(int argc,char** argv){
	ros::init(argc,argv,"img_srv");
	ros::NodeHandle nh;

	ros::ServiceServer service
		= nh.advertiseService("/zed/left/image_rect_color",1,img_srv);

	ros::spin();
	return 0;
}

