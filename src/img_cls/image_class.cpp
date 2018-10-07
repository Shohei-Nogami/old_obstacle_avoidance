#include"image_class.h"

image_class::image_class()
	:it(nh_pub),PROCESS_ONCE(true)
{
	std::cout<<"in image_class constracter\n";
//		  cur_image.reserve();
//		  pre_image.reserve();
//		  debug_image.reserve();
}
bool image_class::is_cur_image(void){
	return (!cur_image.empty());
}
bool image_class::is_pre_image(void){
	return (!pre_image.empty());
}
void image_class::subscribe_image(void){
	queue.callOne(ros::WallDuration(1));
}
void image_class::set_cur_image(void){
	if(!PROCESS_ONCE)
		cur_image=cvbridge_image->image.clone();
}
void image_class::set_delta_time(void){
	delta_time=cur_time-pre_time;
}
void image_class::set_image(void){
	subscribe_image();
	if(is_cur_image()){
		set_pre_image();
	 }
	set_cur_image();
	if(is_pre_image()){

	}
}

cv::Mat& image_class::get_cur_image_by_ref(void){
	return cur_image;
}
cv::Mat& image_class::get_pre_image_by_ref(void){
	return pre_image;
}
double image_class::get_delta_time(void){
	return delta_time.toSec();
}

image_class::~image_class()
{
	cur_image.release();
	pre_image.release();
}

void image_class::set_pre_image(void){
	pre_image=cur_image.clone();
	pre_time=cur_time;
}


void image_class::define_variable(void){
	pub=it.advertise("left_image",1);
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/left/image_rect_color",1,&image_class::image_callback,this);
}


void image_class::image_callback(const sensor_msgs::ImageConstPtr& msg)
{

	try{
		std::cout<<"image_callback \n";
		cvbridge_image=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		cur_time=msg->header.stamp;
		PROCESS_ONCE=false;
	}
	catch(cv_bridge::Exception& e) {
		std::cout<<"image_callback Error \n";
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
		msg->encoding.c_str());
		return ;
	}
}

void image_class::publish_debug_image(cv::Mat& temp_image){
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=temp_image.clone();
	pub.publish(publish_cvimage->toImageMsg());

}

/*
int main(int argc,char **argv){
	ros::init(argc,argv,"image_class_test");
	image_class image;
	std::cout<<"finish\n";
	return 0;
}
*/
