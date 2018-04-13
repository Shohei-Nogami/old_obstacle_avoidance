#include"grid_class.h"

grid_class::grid_class()
	::it(nh_pub),grid_resolution(100),grid_size(5.0)
{
	pub=it.advertise("grid_image",1);
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/point_cloud/cloud_registered",1,&grid_class::plc_callback,this);
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC1);
	grid_map=m.clone();
	grid_cell_size=grid_size/grid_resolution;
}

grid_class::~grid_class(){

}
void grid_class::subscribe_pcl(void){
	queue.callOne(ros::WallDuration(1));
}
void grid_class::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::fromROSMsg (*msg, pcl_data);
	ROS_INFO("into plc_callback");
}
void grid_class::set_grid_map(void){
	
}

void grid_class::publish_grid_map(void){
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::MONO8;
	publish_cvimage->image=grid_map.clone();
	pub.publish(publish_cvimage->toImageMsg());	
}


int main(int argc,char **argv){
	ros::init(argc,argv,"grid_class_test");
	grid_class grid;
	std::cout<<"ready\n";

	while(ros::ok()){
		grid.subscribe_pcl();
		
	}

	return 0;
}
