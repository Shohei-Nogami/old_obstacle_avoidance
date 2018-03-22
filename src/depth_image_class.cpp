#include"depth_image_class.h"


depth_image_class::depth_image_class()
	:FILTER_TYPE(MEDIAN_FILTER)
{
//	filted_cur_image.reserve();
//	filted_pre_image.reserve();
	depth_median.reserve(ksize*ksize);
	//create enpty image with just size
	cv::Mat m = cv::Mat::zeros(cv::Size(width/ksize,height/ksize), CV_32FC1);
	filted_cur_image=m.clone();
}
depth_image_class::~depth_image_class(){
	filted_cur_image.release();
	filted_pre_image.release();
}

void depth_image_class::set_pre_image(void){
	filted_pre_image=filted_cur_image.clone();
}

void depth_image_class::define_variable(void){
	pub=it.advertise("depth_image",1);
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&depth_image_class::image_callback,this);
}

void depth_image_class::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		cvbridge_image=
		cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
	}
  catch(cv_bridge::Exception& e) {
	  ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
	  msg->encoding.c_str());
	  return ;
  }
}

void depth_image_class::filtering_depth_image(void){
  cv::Mat depth_img=get_cur_image_by_ref();
	double depth_sum;
	int depth_element_num;
	FILTER_TYPE=MEDIAN_FILTER;//AVERAGE_FILTER;//NOTHING;
	if(FILTER_TYPE==MEDIAN_FILTER){
//----median filter
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
					filted_cur_image.at<float>(h,w)=depth_median[(int)depth_median.size()/2];
				}
				else{
					filted_cur_image.at<float>(h,w)=0;						
				}
				depth_median.clear();
			}
		}
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
				filted_cur_image.at<float>(h,w)=depth_sum/depth_element_num;						
			}
		}
	}
	else if(FILTER_TYPE==NOTHING){
		for(int h=0;h<height/ksize;h++){
			for(int w=0;w<width/ksize;w++){
				if(std::isnan(depth_img.at<float>(h*ksize,w*ksize))
					||std::isinf(depth_img.at<float>(h*ksize,w*ksize)) ){
					filted_cur_image.at<float>(h,w)=0;						
				}
				else{
					filted_cur_image.at<float>(h,w)=depth_img.at<float>(h*ksize,w*ksize);
				}
			}
		}
	}
}

int main(int argc,char **argv){
	ros::init(argc,argv,"depth_image_class_test");
	depth_image_class depth_image;
	std::cout<<"finish\n";
	return 0;
}



