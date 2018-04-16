#include"grid_class.h"

grid_class::grid_class()
	:it_pub(nh_pub),it_pub2(nh_pub2),grid_resolution(201),grid_size(8.0),EXECUTED_CALLBACK(false),binary_threshold(90)
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

//set parameter of trajectory
	temp_vl.reserve(vel_patern);
	temp_vr.reserve(vel_patern);
	temp_w.reserve(vel_patern);
	temp_p.reserve(vel_patern);
	max_process_n.reserve(vel_patern);
	rank_trajectory.reserve(vel_patern);
	float delta_vel_dif=temp_v_dif_max/(vel_patern-1);
	for(int i=0;i<vel_patern;i++){
		temp_vr.push_back(temp_v-temp_v_dif_max/2+i*delta_vel_dif);
		temp_vl.push_back(temp_v+temp_v_dif_max/2-i*delta_vel_dif);
		temp_w.push_back( (temp_vr[i]-temp_vl[i])/(2*d) );
		if(temp_w[i]!=0){
			double temptemp_p=temp_v/temp_w[i];
			if(temptemp_p<0)
				temp_p.push_back(-temptemp_p);
			else
				temp_p.push_back(temptemp_p);				
		}
		else{
			temp_p.push_back(0);
		}
		delta_theta.push_back(R/temp_p[i]);
		max_process_n.push_back((int)(2*M_PI/delta_theta[i]));
	}
//set parameter of collision avoidance
	int n_circle_size=2*(int)( 2*(R+d_r)/grid_cell_size / 2 ) + ( (int)(2*(R+d_r)/grid_cell_size) / 2 ) % 2; 
	jn_Rd=n_circle_size;
	in_Rd.reserve(n_circle_size);
	double temp_x;
	for(int j=0;j<jn_Rd;j++){
		temp_x=sqrt( (R+d_r)*(R+d_r) - ((j-jn_Rd/2)*grid_cell_size)*((j-jn_Rd/2)*grid_cell_size) );
		in_Rd.push_back( 
			(int)( 2*(temp_x)/grid_cell_size / 2 ) + ( (int)(2*(temp_x)/grid_cell_size)  ) % 2 );
	}
//debug
	for(int i=0;i<vel_patern;i++){
		std::cout<<"vr,vl,w,p,dtheta,:"<<temp_vr[i]<<","<<temp_vl[i]<<","<<temp_w[i]<<","<<temp_p[i]<<","<<delta_theta[i]<<","<<max_process_n[i]<<"\n";
	}
	std::cout<<"R+d_r,int(R+d_r),Sc:"<<R+d_r<<","<<n_circle_size<<","<<grid_cell_size<<"\n";
	for(int j=0;j<jn_Rd;j++){
		std::cout<<"in_Rd["<<j<<"]:"<<in_Rd[j]<<"\n";
	}
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
				y=(height/2-h)*depth_temp/f;//for estimate floor
				if(y+0.23>floor_threshold){//||y_init+0.23>1.0){
					if(depth_temp<grid_size/2){
						grid_z=(int)( (grid_size/2-depth_temp) /grid_cell_size);
						x=(w-width/2)*depth_temp/f;
						if(std::abs(x)<grid_size/2){
//							grid_x=(int)( (x+grid_size/2)/grid_cell_size);
							grid_x=( (int)(2*(x+grid_size/2)/grid_cell_size)/2 ) + ( (int)(2*(x+grid_size/2)/grid_cell_size) ) % 2;
//							grid_map.at<uchar>(grid_z,grid_x)=255;
							grid_map.at<uint8_t/*uchar*/>(grid_z,grid_x)+=2;
							if(grid_map.at<uint8_t/*uchar*/>(grid_z,grid_x)>=255){
								grid_map.at<uint8_t/*uchar*/>(grid_z,grid_x)=255;
							}
						}
					}
				}
			}
		}
	}

	grid_map-=1;

}

void grid_class::select_best_trajectory(void){
	double xr;
	double yr;
	int nx;
	int ny;
	rank_trajectory.clear();

	for(int i=0;i<temp_p.size();i++){
		double theta=0;
		int j_max=15;//test param
		int j=0;
		bool LOOP_OUT=false;
		for(/*int j=0*/;j<j_max && (j<max_process_n[i] || i==temp_p.size()/2);j++,theta+=delta_theta[i]){
			if(temp_w[i]>0){
				xr=temp_p[i]*(cos(delta_theta[i]*j)-1);
				yr=temp_p[i]*sin(delta_theta[i]*j);
			}
			else if(temp_w[i]<0){
				xr=temp_p[i]*(1-cos(delta_theta[i]*j));
				yr=temp_p[i]*sin(delta_theta[i]*j);
			}
			else{
				xr=0;
				yr=j*R;
			}
//			if(i==7)
//				std::cout<<"max_process_n[i]:j::"<<max_process_n[i]<<","<<j<<"\n";
			transport_robot_to_gridn(xr,yr,nx,ny);
			if( is_obstacle(nx,ny) ){
				LOOP_OUT=true;
				rank_trajectory.push_back(j);
				break;
			}
		}
		if(!LOOP_OUT)
			rank_trajectory.push_back(j);		
	}
	float good_trajectory_value=0;
	int good_trajectory_num=0;
	float evaluation_formula;
	for(int i=0;i<vel_patern;i++){
		evaluation_formula=rank_trajectory[i];
		std::cout<<"trajectory["<<i<<"]:"<<evaluation_formula<<"\n";

		if(good_trajectory_value<evaluation_formula){
			good_trajectory_value=evaluation_formula;
			good_trajectory_num=i;
		}
	}
	std::cout<<"good tarjectory is "<<good_trajectory_num<<"\n";
	std::cout<<"good tarjectory value is "<<good_trajectory_value<<"\n";
	draw_best_trajectory(good_trajectory_num);
}
void grid_class::transport_robot_to_gridn(const double& xr,const double& yr,int& n_xr,int& n_yr){
	double grid_x=xr+grid_size/2;
	double grid_y=grid_size/2-yr;

	transport_gridx_to_gridn(grid_x,grid_y,n_xr,n_yr);
}

void grid_class::transport_gridx_to_gridn(const double& x,const double& y,int& n_x,int& n_y){
	n_x = (int)(2*x/grid_cell_size/2) + (int)(2*x/grid_cell_size) % 2;
	n_y = (int)(2*y/grid_cell_size/2) + (int)(2*y/grid_cell_size) % 2;
}

bool grid_class::is_obstacle(const int nx,const int ny){

	for(int j=0;j<jn_Rd;j++){
		for(int i=-in_Rd[j]+nx;i<=in_Rd[j]+nx;i++){
			if((j-jn_Rd/2)+ny<0 || (j-jn_Rd/2)+ny>grid_resolution || i<0 || i>grid_resolution)
				return true;
			if(grid_map.at<uint8_t>((j-jn_Rd/2)+ny,i)>=	binary_threshold)
				return true;
		}
	}
	return false;
}
void grid_class::draw_best_trajectory(const int& num){

	double good_p=temp_p[num];
	double good_delta_theta=delta_theta[num];
	double good_w=temp_w[num];
	int nx,ny;
	int nx_1=0;
	int ny_1=0;
	double xr=0;
	double yr=0;
	double theta=0;
	int j_max=15;//test param
	for(int j=0;j<j_max && (j<max_process_n[num] || num==temp_p.size()/2) ;j++,theta+=good_delta_theta){
		if(good_w>0){
			xr=good_p*(cos(good_delta_theta*j)-1);
			yr=good_p*sin(good_delta_theta*j);
		}
		else if(good_w<0){
			xr=good_p*(1-cos(good_delta_theta*j));
			yr=good_p*sin(good_delta_theta*j);
		}
		else{
			xr=0;
			yr=j*R;
		}
		transport_robot_to_gridn(xr,yr,nx,ny);
		if(j>0&&nx_1!=nx&&ny_1!=ny){
			cv::line(grid_map_view, cv::Point(nx_1, ny_1), cv::Point(nx, ny), cv::Scalar(0,255,255), 1, 4);	
		}
		nx_1=nx;
		ny_1=ny;
	}
	
}
void grid_class::draw_all_trajectory(void){
	double xr;
	double yr;
	int nx;
	int ny;
	int nx_1;
	int ny_1;
	for(int i=0;i<temp_p.size();i++){
		double theta=0;
		int j_max=15;//test param
		int j=1;
		nx_1=0;
		ny_1=0;
		nx=0;
		ny=0;
//		std::cout<<"i,(int)(2*M_PI/delta_theta[i]):"<<i<<","<<(int)(2*M_PI/delta_theta[i])<<"\n";
//		std::cout<<"i:"<<i<<"\n";
		for(/*int j=0*/;j<j_max && (j<max_process_n[i] || i==temp_p.size()/2) ;j++,theta+=delta_theta[i]){
			if(temp_w[i]>0){
				xr=temp_p[i]*(cos(delta_theta[i]*j)-1);
				yr=temp_p[i]*sin(delta_theta[i]*j);
			}
			else if(temp_w[i]<0){
				xr=temp_p[i]*(1-cos(delta_theta[i]*j));
				yr=temp_p[i]*sin(delta_theta[i]*j);
			}
			else{
//				std::cout<<"else in i:"<<i<<"\n";
				xr=0;
				yr=j*R;
			}
//			std::cout<<"-j,(x,y):"<<j<<",("<<xr<<","<<yr<<")\n";

			transport_robot_to_gridn(xr,yr,nx,ny);
			if( is_obstacle(nx,ny) ){
//				std::cout<<"i,j:break::"<<i<<","<<j<<"\n";
				break;
			}
			else{
//				std::cout<<"draw line\n";
				if(nx_1!=nx&&ny_1!=ny)
					cv::line(grid_map_view, cv::Point(nx_1, ny_1), cv::Point(nx, ny), cv::Scalar(0,255,255), 1, 4);	
			}
			nx_1=nx;
			ny_1=ny;
		}	
	}

}


void grid_class::set_grid_map_view(void){
	for(int h=0;h<grid_resolution;h++){
		for(int w=0;w<grid_resolution;w++){
			grid_map_view.at<cv::Vec3b>(h,w)[1]=grid_map.at<uchar>(h,w);	
			grid_map_view.at<cv::Vec3b>(h,w)[0]=0;	
			grid_map_view.at<cv::Vec3b>(h,w)[2]=0;	
					
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

	float robot_r=R;
	int robot_cell_size=(int)(robot_r/grid_cell_size);
	for(int h=grid_resolution/2-robot_cell_size;h<grid_resolution/2+robot_cell_size;h++){
		for(int w=grid_resolution/2-robot_cell_size;w<grid_resolution/2+robot_cell_size;w++){
			grid_map_view.at<cv::Vec3b>(h,w)[2]=255;			
		}
	}

	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=grid_map_view.clone();
	pub.publish(publish_cvimage->toImageMsg());	
}
void grid_class::publish_binary_grid_map_view(void){


	float robot_r=R;
	int robot_cell_size=(int)(robot_r/grid_cell_size);
/*
	for(int h=grid_resolution/2-robot_cell_size/2;h<grid_resolution/2+robot_cell_size/2;h++){
		for(int w=grid_resolution/2-robot_cell_size/2;w<grid_resolution/2+robot_cell_size/2;w++){
			binary_grid_map_view.at<cv::Vec3b>(h,w)[2]=255;			
		}
	}
*/
	int cp_grid_map=grid_resolution/2;
	for(int j=0;j<jn_Rd;j++){
		for(int i=-in_Rd[j]+grid_resolution/2;i<=in_Rd[j]+grid_resolution/2;i++){
			binary_grid_map_view.at<cv::Vec3b>((j-jn_Rd/2)+grid_resolution/2,i)[2]=255;	
		}
	} 
	for(int h=grid_resolution/2-robot_cell_size;h<grid_resolution/2+robot_cell_size;h++){
		for(int w=grid_resolution/2-robot_cell_size;w<grid_resolution/2+robot_cell_size;w++){
			binary_grid_map_view.at<cv::Vec3b>(h,w)[0]=255;
			binary_grid_map_view.at<cv::Vec3b>(h,w)[2]=0;		
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
			grid.select_best_trajectory();
//			grid.draw_all_trajectory();
			grid.publish_grid_map_view();
			grid.set_binary_grid_map_view();
			grid.publish_binary_grid_map_view();
		}
	}

	return 0;
}

