#include"image_process_zed_function.h";

bool dpt_srv(const sensor_msgs::ImageConstPtr& dpt)
{
	try{
		depthimg = cv_bridge::toCvCopy(dpt,
			 sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch(cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
		dpt->encoding.c_str());
		return false;
	}
	return true;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"dpt_srv");
	ros::NodeHandle nh;

	ros::ServiceServer dpt_srv 
		= n.advertiseService("/zed/depth/depth_registered",dpt_srv);

	ros::spin();
	return 0;
}
