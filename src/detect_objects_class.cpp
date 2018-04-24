#include"detect_objects_class.h"
#include"time_class.h"


detect_objects::detect_objects()
	:it_pub1(nh_pub1),it_pub2(nh_pub2),EXECUTED_CALLBACK(false),cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
	
	pub1=it_pub1.advertise("detected_objects_image",1);//test string
	pub2=it_pub2.advertise("detected_objects_image2",1);//test string
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&detect_objects::image_callback,this);

	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			dem_element[nz][nx].reserve(width);
		}
	}
  pc_pub = nh_pubpcl.advertise<sensor_msgs::PointCloud2>("edit_cloud", 1);
  cloud->width  = width;
  cloud->height = height;
  cloud->points.resize (cloud->width * cloud->height);

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
void detect_objects::set_depth_image(void){
	depth_image=cvbridge_image->image.clone();
}

bool detect_objects::convert_coordinate_xz_nxz(const float x,const float z,int& nx,int& nz){
	nx = (int)(2*x/cell_size/2) + (int)(2*x/cell_size) % 2;
	nz = (int)(2*z/cell_size/2) + (int)(2*z/cell_size) % 2;
	if(map_size_nx<std::abs(nx)||map_size_nz<nz){
		return false;
	}
	else{
		nx+=map_size_nx/2;
		nz=map_size_nz-nz;
		return true;
	}
}
void detect_objects::invert_coordinate_xz_nxz(const int& nx,const int& nz,float& x,float& z){
	x=(nx-map_size_nx/2)*cell_size;
	z=(map_size_nz-nz)*cell_size;

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
			if(z_temp>0.5&&!std::isinf(z_temp)){//
				y_temp=(height/2-h)*z_temp/f;
				if(y_temp+0.23>ground_threshold){		
					x_temp=(w-width/2)*z_temp/f;
					if(convert_coordinate_xz_nxz(x_temp,y_temp,nx,nz)){
						dem_element[nz][nx].push_back(y_temp);
					}
				}
			}
		}
	}
}
void detect_objects::clustering_DEM_elements(void){

	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
//			dem_element[nz][nx].sort(dem_element[nz][nx].begin(),dem_element[nz][nx].end());
			std::sort(dem_element[nz][nx].begin(),dem_element[nz][nx].end());
		}
	}
//	std::cout<<"did sort\n";
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
//			if(dem_element[nz][nx].size()){
//				dem_cluster[nz][nx].push_back(dem_element[nz][nx][0]);
//			}
			int iy_low=0;
			int iy_high=0;
//			std::cout<<"processing in (nz,nx),size:("<<nz<<","<<nx<<"),"
//				<<dem_element[nz][nx].size()<<"\n";
//			std::cout<<"(int)size-1="<<(int)dem_element[nz][nx].size()-1<<"\n";
			if(dem_element[nz][nx].size()){
//				std::cout<<"processing in (nz,nx),size:("<<nz<<","<<nx<<"),"				
//					<<dem_element[nz][nx].size()<<"\n";
//				std::cout<<"(z,x):("<<(map_size_nz-nz)*cell_size<<","<<(nx-map_size_nx/2)*cell_size<<"\n";
			}
			for(int i=0;i<(int)dem_element[nz][nx].size()-1;i++){
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
	
	std::vector< std::vector<cv::Point2i> > slice_cluster[map_size_nz];
	std::vector<cv::Point2i> slice_cluster_element;
	cv::Point2i tp;//x:i,y:k
	std::vector<int> clusted_flag[map_size_nz][map_size_nx];
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			clusted_flag[nz][nx].reserve((int)dem_cluster[nz][nx].size());					
			for(int k=0;k<(int)dem_cluster[nz][nx].size();k++){
				clusted_flag[nz][nx].push_back(-1);
				
			}
		}
	}
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int k=0;k<(int)dem_cluster[nz][nx].size();k++){
				if(clusted_flag[nz][nx][k]!=-1){
					continue;
				}
				tp.x=nx;
				tp.y=k;
				slice_cluster_element.push_back(tp);
				clusted_flag[nz][nx][k]=(int)slice_cluster[nz].size()+1;//be careful to transform the struct
				for(int k0=0;nx<map_size_nx&&k0<(int)slice_cluster_element.size();k0++){
					int x0=slice_cluster_element[k0].x;
					int kk=slice_cluster_element[k0].y;
					for(int k1=0;k1<(int)dem_cluster[nz][x0+1].size();k1++){
//					if dem_cluster[nz][x0][kk] touched dem_cluster[nz][x0+1][k1];
						if(dem_cluster[nz][x0][kk].x<=dem_cluster[nz][x0+1][k1].y
							&&dem_cluster[nz][x0][kk].y>=dem_cluster[nz][x0+1][k1].x){
							if(clusted_flag[nz][x0][k1]!=-1){
								tp.x=x0;
								tp.y=k1;
								slice_cluster_element.push_back(tp);
								clusted_flag[nz][x0][k1]=(int)slice_cluster[nz].size()+1;
							}
							else{
								if(clusted_flag[nz][x0][k1]!=(int)slice_cluster[nz].size()+1){
									tp.x=x0;
									tp.y=k1;
									slice_cluster[nz][ clusted_flag[nz][x0][k1] ].push_back(tp);
								}
							}//end else
						}//end if touched
					}//end for k1
				}//end for k0
			}//end for k
		}//end for nx
	}//end for nz
}

void detect_objects::convert_dem_to_pcl(void){
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout<<"in conv dem to pcl\n";

	cloud->clear();
	int points_size=0;
	int count=0;
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			points_size+=(int)dem_element[nz][nx].size();
			for(int k=0;k<(int)dem_element[nz][nx].size();k++){
		    pcl::PointXYZ point;
				invert_coordinate_xz_nxz(nx,nz,point.x,point.z);
		    point.y = dem_element[nz][nx][k];
		    cloud->points[count++]=point;
			}
    }
  }
//	std::cout<<"points_size:"<<points_size<<"\n";
//	std::cout<<"finished push_back\n";
	cloud->width=count;//points_size;
	cloud->height=1;
  cloud->points.resize (cloud->width * cloud->height);
	std::cout<<"resized size:"<<cloud->width * cloud->height<<"\n";
//
//	points_size+=(int)dem_element[nz][nx].size();
/*
	std::cout<<",,,,\n";
	cloud->clear();
	cloud->width=100;
	cloud->height=100;
  cloud->points.resize (cloud->width * cloud->height);
	float reso=0.01;
	int count=0;
	for(int i=0;i<100;i++){
		for(int j=0;j<100;j++){
			pcl::PointXYZ point;
			point.x=i*reso;
			point.y=j*reso;
			point.z=1;
	    cloud->points[count++]=point;
		}
	}
	std::cout<<"cloud width*height:"<<cloud->width * cloud->height<<"\n";
	std::cout<<"cloud size,count:"<<cloud->points.size()<<","<<count<<"\n";
*/	

  sensor_msgs::PointCloud2 edit_cloud;
  pcl::toROSMsg (*cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
//	edit_cloud.header.stamp=ros::Time::now();
//	edit_cloud.header.seq+=1;
  pc_pub.publish(edit_cloud);

//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");  
//  viewer.showCloud (cloud);//.makeShared()); 
//	while (!viewer.wasStopped ()) {  
//  }
	std::cout<<"published\n";
//	cloud->points.clear();
	cloud->clear();
//	std::cout<<"cleared size:"<<cloud->width * cloud->height<<"\n";
  cloud->width  = width;
  cloud->height = height;
  cloud->points.resize (cloud->width * cloud->height);

	
}
void detect_objects::clear_dem_element(void){
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			dem_element[nz][nx].clear();
		}
	}

}

int main(int argc,char **argv){
	ros::init(argc,argv,"detect_objects_class_test");
	detect_objects dtct_obj;
  time_class time_cls;
	std::cout<<"defined class\n";
	float x,z;
	int nx,nz;
	bool tf;
	while(ros::ok()){
	/*
	std::cout<<"x:";std::cin>>x;
	std::cout<<"z:";std::cin>>z;
	tf=dtct_obj.convert_coordinate_xz_nxz(x,z,nx,nz);
	if(tf){
		std::cout<<"trans is success\n";
		std::cout<<"nx,nz:"<<nx<<","<<nz<<"\n";
		std::cout<<"(nx-cx)*cs,(mz-nz)*cs:"<<(nx-201/2)*0.04
			<<","<<(401-nz)*0.04<<"\n";
	}
	else{
		std::cout<<"trans is failed\n";
	}
	*/

	
		dtct_obj.subscribe_depth_image();
		if(!dtct_obj.is_cvbridge_image())
			continue;
//		std::cout<<"subsctibed cvbridgeimage\n";
		dtct_obj.set_depth_image();
//		std::cout<<"set matimage\n";
		dtct_obj.set_DEM_map();
//		std::cout<<"set demmap\n";
//		dtct_obj.clustering_DEM_elements();
//		std::cout<<"now processing!\n";
		dtct_obj.convert_dem_to_pcl(); 
		dtct_obj.clear_dem_element();
		time_cls.set_time();
		std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	
	}

	return 0;
}


