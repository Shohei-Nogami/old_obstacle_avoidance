#include"grid_class.h"

grid_class::grid_class()
	:it_pub(nh_pub),it_pub2(nh_pub2),grid_resolution(200),grid_size(8.0),EXECUTED_CALLBACK(false),binary_threshold(90)
{
	pub=it_pub.advertise("grid_image",1);
	pub2=it_pub2.advertise("binary_grid_image",1);
	nh_sub.setCallbackQueue(&queue);
//	sub=nh_sub.subscribe("/zed/point_cloud/cloud_registered",1,&grid_class::plc_callback,this);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&grid_class::image_callback,this);
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC1);
	grid_map=m.clone();
	cv::Mat m_view = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC3);	
	grid_map_view=m_view.clone();
	binary_grid_map_view=m_view.clone();
	grid_cell_size=grid_size/grid_resolution;
}

grid_class::~grid_class(){

}
void grid_class::subscribe_depth_image(void){
	queue.callOne(ros::WallDuration(1));
}
/*
void grid_class::subscribe_pcl(void){
	queue.callOne(ros::WallDuration(1));
}
void grid_class::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::fromROSMsg (*msg, pcl_data);
	ROS_INFO("into plc_callback");
}
*/
void grid_class::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
//		std::cout<<"depth_image_callback \n";
		cvbridge_image=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
		EXECUTED_CALLBACK=true;
	}
  catch(cv_bridge::Exception& e) {
	std::cout<<"depth_image_callback Error \n";
	  ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
	  msg->encoding.c_str());
	  return ;
  }
}
bool grid_class::is_cvbridge_image(void){
	return EXECUTED_CALLBACK;
}
void grid_class::set_grid_map(void){
	float x,y;
	int grid_x,grid_z;
	float depth_temp;
	const float floor_threshold=0.10;
	depth_image=cvbridge_image->image.clone();

//renew
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC1);
	grid_map=m.clone();

	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			depth_temp=depth_image.at<float>(h,w);
			if(!std::isnan(depth_temp)&&!std::isinf(depth_temp)){
				y=(height/2-h)*depth_temp/f;
				if(y+0.23>floor_threshold){//||y_init+0.23>1.0){
					if(depth_temp<grid_size/2){
							grid_z=(int)( (grid_size/2-depth_temp) /grid_cell_size);
						x=(w-width/2)*depth_temp/f;
						if(std::abs(x)<grid_size/2){
							grid_x=(int)( (x+grid_size/2)/grid_cell_size);
//							grid_map.at<uchar>(grid_z,grid_x)=255;
							grid_map.at<uchar>(grid_z,grid_x)+=2;
							if(grid_map.at<uchar>(grid_z,grid_x)>=255){
								grid_map.at<uchar>(grid_z,grid_x)=255;
							}
						}
					}
				}
			}
		}
	}

	grid_map-=1;

}
void grid_class::set_grid_map_view(void){
	for(int h=0;h<grid_resolution;h++){
		for(int w=0;w<grid_resolution;w++){
			grid_map_view.at<cv::Vec3b>(h,w)[1]=grid_map.at<uchar>(h,w);			
		}
	}
}
void grid_class::set_binary_grid_map_view(void){
	for(int h=0;h<grid_resolution;h++){
		for(int w=0;w<grid_resolution;w++){
			if(grid_map.at<uchar>(h,w)>=binary_threshold)
				binary_grid_map_view.at<cv::Vec3b>(h,w)[1]=255;
			else
				binary_grid_map_view.at<cv::Vec3b>(h,w)[1]=0;
		}
	}
} 
void grid_class::publish_grid_map_view(void){

	float robot_r=0.20;
	int robot_cell_size=(int)(robot_r/grid_cell_size);
	for(int h=grid_resolution/2-robot_cell_size/2;h<grid_resolution/2+robot_cell_size/2;h++){
		for(int w=grid_resolution/2-robot_cell_size/2;w<grid_resolution/2+robot_cell_size/2;w++){
			grid_map_view.at<cv::Vec3b>(h,w)[2]=255;			
		}
	}

	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=grid_map_view.clone();
	pub.publish(publish_cvimage->toImageMsg());	
}
void grid_class::publish_binary_grid_map_view(void){

	float robot_r=0.20;
	int robot_cell_size=(int)(robot_r/grid_cell_size);
	for(int h=grid_resolution/2-robot_cell_size/2;h<grid_resolution/2+robot_cell_size/2;h++){
		for(int w=grid_resolution/2-robot_cell_size/2;w<grid_resolution/2+robot_cell_size/2;w++){
			binary_grid_map_view.at<cv::Vec3b>(h,w)[2]=255;			
		}
	}

	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=binary_grid_map_view.clone();
	pub2.publish(publish_cvimage->toImageMsg());	
}

int main(int argc,char **argv){
	ros::init(argc,argv,"grid_class_test");
	grid_class grid;
	std::cout<<"ready\n";

	while(ros::ok()){
		grid.subscribe_depth_image();
		if(grid.is_cvbridge_image()){
			grid.set_grid_map();
			grid.set_grid_map_view();
			grid.publish_grid_map_view();
			grid.set_binary_grid_map_view();
			grid.publish_binary_grid_map_view();
		}
	}

	return 0;
}

