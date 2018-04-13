#include"grid_class.h"

grid_class::grid_class()
	:it_pub(nh_pub),grid_resolution(200),grid_size(8.0)
{
	pub=it_pub.advertise("grid_image",1);
	nh_sub.setCallbackQueue(&queue);
//	sub=nh_sub.subscribe("/zed/point_cloud/cloud_registered",1,&grid_class::plc_callback,this);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&grid_class::image_callback,this);
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC3);
	grid_map=m.clone();
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
	}
  catch(cv_bridge::Exception& e) {
	std::cout<<"depth_image_callback Error \n";
	  ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
	  msg->encoding.c_str());
	  return ;
  }
}
void grid_class::set_grid_map(void){
	float x,y;
	int grid_x,grid_z;
	float depth_temp;
	const float floor_threshold=0.10;
	depth_image=cvbridge_image->image.clone();
//renew
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC3);
	grid_map=m.clone();

//	std::cout<<"before for\n";
	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			depth_temp=depth_image.at<float>(h,w);
//			std::cout<<"("<<h<<","<<w<<")\n";
			if(!std::isnan(depth_temp)&&!std::isinf(depth_temp)){
//				std::cout<<"depth_temp:"<<depth_temp<<"\n";
				y=(height/2-h)*depth_temp/f;
				if(y+0.23>floor_threshold){//||y_init+0.23>1.0){
					if(depth_temp<grid_size/2){
	//					std::cout<<"depth_temp<grid_size/2\n";
	//					std::cout<<"depth_temp/grid_cell_size:"<<(int)(depth_temp/grid_cell_size)<<"\n";
	//					std::cout<<"grid_resolution/2:"<<grid_resolution/2<<"\n";
	//					std::cout<<"grid_resolution/2-(int)(depth_temp/grid_cell_size):"<<grid_resolution/2-(int)(depth_temp/grid_cell_size)<<"\n";
//						grid_z=grid_resolution/2-(int)(depth_temp/grid_cell_size);
							grid_z=(int)( (grid_size/2-depth_temp) /grid_cell_size);
	//					std::cout<<"grid:(z,x):("<<grid_z;
	//					std::cout<<"a1:\n";
						x=(w-width/2)*depth_temp/f;
	//					std::cout<<"a2\n";
						if(std::abs(x)<grid_size/2){
//							grid_x=(int)(x/grid_cell_size)+grid_resolution/2;
							grid_x=(int)( (x+grid_size/2)/grid_cell_size);
	//						std::cout<<","<<grid_x;
//							grid_map.at<cv::Vec3b>(grid_z,grid_x)[1]+=2;
							grid_map.at<cv::Vec3b>(grid_z,grid_x)[1]=255;
//							if(grid_map.at<cv::Vec3b>(grid_z,grid_x)[1]>=255){
//								grid_map.at<cv::Vec3b>(grid_z,grid_x)[1]=255;
//							}
	//						std::cout<<")\n";
						}
					}
				}
			}
		}
	}
/*
	for(int h=0;h<grid_resolution;h++){
		for(int w=0;w<grid_resolution;w++){
			grid_map.at<int>(h,w)-=1;				
			if(grid_map.at<int>(h,w)<=0){
				grid_map.at<int>(h,w)=0;
			}
		}
	}
*/
	grid_map-=5;

}

void grid_class::publish_grid_map(void){

	float robot_r=0.20;
	int robot_cell_size=(int)(robot_r/grid_cell_size);
	for(int h=grid_resolution/2-robot_cell_size/2;h<grid_resolution/2+robot_cell_size/2;h++){
		for(int w=grid_resolution/2-robot_cell_size/2;w<grid_resolution/2+robot_cell_size/2;w++){
			grid_map.at<cv::Vec3b>(h,w)[2]=255;			
		}
	}

//	std::cout<<"into pulish func\n";
/*	for(int h=0;h<grid_resolution;h++){
		for(int w=0;w<grid_resolution;w++){
//			std::cout<<"("<<h<<","<<w<<")\n";
			grid_map.at<cv::Vec3b>(h,w)[1]=255;			
		}
	}
	std::cout<<"a1\n";
*/	

	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=grid_map.clone();
	pub.publish(publish_cvimage->toImageMsg());	
}


int main(int argc,char **argv){
	ros::init(argc,argv,"grid_class_test");
	grid_class grid;
	std::cout<<"ready\n";

	while(ros::ok()){
		grid.subscribe_depth_image();
		grid.set_grid_map();
//		std::cout<<"setted grid_map\n";
		grid.publish_grid_map();
	}

	return 0;
}

