#include"detect_objects_class.h"



detect_objects::detect_objects()
	:it_pub(nh_pub),it_pub2(nh_pub2),EXECUTED_CALLBACK(false)
{
	
	pub=it_pub.advertise("detected_objects_image",1);//test string
	pub2=it_pub2.advertise("detected_objects_image2",1);//test string
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&grid_class::image_callback,this);



}
detect_objects::~detect_objects(){

}

void detect_objects::subscribe_depth_image(void){
	queue.callOne(ros::WallDuration(1));
}
void detect_objects::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
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
bool detect_objects::is_cvbridge_image(void){
	return EXECUTED_CALLBACK;
}

void detect_objects::set_DEM_map(void){

	const float ground_threshold=0.10;

	float z_temp,x_temp,y_temp;
	int cam_nx=map_size_nx/2;
	int cam_nz=map_size_nz-1;
	int nx,nz;
	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			z_temp=depth_image.at<float>(h,w);
			if(!std::isnan(z_temp)&&!std::isinf(z_temp)){
				y_temp=f*(height/2-h)/z_temp;
				if(y+0.23>ground_threshold){		
					x_temp=f*(w-width/2)/z_temp;
					transrate_coordinate_xz_nxz(x_temp,y_temp,nx,nz)
					dem_element[nz][nx].push_back(y_temp);
				}
			}
		}
	}

}
void detect_objects::clustering_DEM_elements(void){

	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			dem_element[nz][nx].sort(dem_element[nz][nx].begin(),dem_element[nz][nx].end());
		}
	}
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
//			if(dem_element[nz][nx].size()){
//				dem_cluster[nz][nx].push_back(dem_element[nz][nx][0]);
//			}
			int iy_low=0;
			int iy_high=0;
			for(int i=0;i<dem_element[nz][nx].size()-1;i++){
				if(dem_element[nz][nx][i+1]-dem_element[nz][nx][i]<h_th){
					iy_high++;
				}
				else{
					cv::Point2f y_low_high;
					y_low_high.x=dem_element[nz][nx][iy_low];//low
					y_low_high.y=dem_element[nz][nx][iy_high];//high

					dem_cluster[nz][nx].push_back(y_low_high);

					iy_low=iy_high+1;
					iy_high=iy_high+1;
				}	
			}
		}
	}
}

void detect_objects::clustering_slice(void){
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			
		}
	}
}

void detect_objects::transrate_coordinate_xz_nxz(const float x,const float z,int& nx,int& nz){
	nx = (int)(2*x/cell_size/2) + (int)(2*x/cell_size) % 2;
	nz = (int)(2*z/cell_size/2) + (int)(2*z/cell_size) % 2;
}




