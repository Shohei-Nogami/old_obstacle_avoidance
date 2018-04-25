#include"detect_objects_class.h"
#include"time_class.h"


detect_objects::detect_objects()
	:it_pub1(nh_pub1),it_pub2(nh_pub2),EXECUTED_CALLBACK(false),cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud2(new pcl::PointCloud<pcl::PointXYZ>),voxeled_cloud (new pcl::PointCloud<pcl::PointXYZ>),ground_deleted_cloud (new pcl::PointCloud<pcl::PointXYZ>),inliers (new pcl::PointIndices),coefficients(new pcl::ModelCoefficients)//,Eclusted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
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
  pc_pub = nh_pubpcl.advertise<sensor_msgs::PointCloud2>("dem_cloud", 1);
  pc_pub2 = nh_pubpcl2.advertise<sensor_msgs::PointCloud2>("trans_image_to_cloud", 1);
  pc_pub3 = nh_pubpcl3.advertise<sensor_msgs::PointCloud2>("vocel_cloud", 1);
  pc_pub4 = nh_pubpcl4.advertise<sensor_msgs::PointCloud2>("ground_delete_cloud", 1);
  pc_pub5 = nh_pubpcl5.advertise<sensor_msgs::PointCloud2>("Eclusted_cloud", 1);
//  cloud->width  = width;
//  cloud->height = height;
//  cloud->points.resize (cloud->width * cloud->height);

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
	x=-((float)nx-(float)map_size_nx/2)*cell_size;
	z=((float)map_size_nz-(float)nz)*cell_size;

}
void detect_objects::convet_image_to_pcl(void){
//	std::cout<<"1\n";
	cloud2->width  = width;
  cloud2->height = height;
	
  cloud2->points.resize (cloud2->width * cloud2->height);
	const float ground_threshold=0;	
	float z_temp,x_temp,y_temp;
	float cam_x=0.12/2;
	int k=0;
	//std::cout<<"width,width/2:"<<width<<","<<width/2<<"\n";
	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			z_temp=depth_image.at<float>(h,w);
			if(z_temp>0.5&&!std::isinf(z_temp)){//
				y_temp=((float)height/2-(float)h)*z_temp/f;
				if(y_temp+0.23>ground_threshold&&y_temp+0.23<1.5){
					x_temp=-( ((float)w-(float)width/2)*z_temp/f-cam_x );
					cloud2->points[k].x=z_temp;
					cloud2->points[k].y=x_temp;
					cloud2->points[k++].z=y_temp;
				}
			}
		}
	}
	cloud2->width  = k;
  cloud2->height = 1;
  cloud2->points.resize (cloud2->width * cloud2->height);
	std::cout<<"original size:"<< cloud2->points.size()<<"\n";
/*
  sensor_msgs::PointCloud2 edit_cloud;
  pcl::toROSMsg (*cloud2, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
  pc_pub2.publish(edit_cloud);
*/

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud2);
	vg.setLeafSize (0.015f, 0.015f, 0.015f);
	vg.filter (*voxeled_cloud);
	std::cout<<"voxceled size:"<< voxeled_cloud->points.size()<<"\n";

  sensor_msgs::PointCloud2 edit_cloud3;
  pcl::toROSMsg (*voxeled_cloud, edit_cloud3);
	edit_cloud3.header.frame_id="/zed_current_frame";
  pc_pub3.publish(edit_cloud3);

/*	
	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出

	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (500);//RANSACの繰り返し回数
	seg.setDistanceThreshold (0.1);//モデルとどのくらい離れていてもいいか???謎
	seg.setAxis(Eigen::Vector3f (0.0,0.0,1.0));//法線ベクトル
	seg.setEpsAngle(20.0f * (M_PI/180.0f));//許容出来る平面の傾きラジアン

	seg.setInputCloud (voxeled_cloud);
*/
/*	coefficients->values[0]=-0.25;
	coefficients->values[1]=0;
	coefficients->values[2]=0;
	coefficients->values[3]=0.388003;
*/	
//	seg.segment (*inliers, *coefficients);

//	ground_deleted_cloud->points.resize(cloud2->points.size());


	ground_deleted_cloud->width=voxeled_cloud->points.size();
  ground_deleted_cloud->height = 1;
	ground_deleted_cloud->points.resize(ground_deleted_cloud->width*ground_deleted_cloud->height);
//	ground_deleted_cloud->points.reserve(ground_deleted_cloud->width);
	
	for(int k=0;k<voxeled_cloud->points.size();k++){
//		y_temp=(0.20*voxeled_cloud->points[k].x-0.388)/0.95;
		y_temp=(0.08*voxeled_cloud->points[k].x-0.388)/0.95;
//		std::cout<<"y_temp:"<<y_temp<<"\n";
		if(y_temp+0.23<voxeled_cloud->points[k].z){
			ground_deleted_cloud->points[k]=voxeled_cloud->points[k];
//			ground_deleted_cloud->points.push_back(voxeled_cloud->points[k]);
//			cloud2->points[k].x;
//			cloud2->points[k].y;
//			cloud2->points[k].z;
		}
	}
	ground_deleted_cloud->width=ground_deleted_cloud->points.size();
  ground_deleted_cloud->height = 1;
	std::cout<<"ground_deleted_cloud->width:"<<ground_deleted_cloud->width<<"\n";
	ground_deleted_cloud->points.resize(ground_deleted_cloud->width*ground_deleted_cloud->height);	
/*
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (voxeled_cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);//true:平面を削除、false:平面以外削除
	extract.filter (*ground_deleted_cloud);
  std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
*/
	sensor_msgs::PointCloud2 edit_cloud4;
  pcl::toROSMsg (*ground_deleted_cloud, edit_cloud4);
	edit_cloud4.header.frame_id="/zed_current_frame";
  pc_pub4.publish(edit_cloud4);

	std::cout<<"publish ground_deleted_cloud\n";
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//何か探索用にツリーを作る
	tree->setInputCloud (ground_deleted_cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02);//同じクラスタとみなす距離
	ec.setMinClusterSize (100);//クラスタを構成する最小の点数
	ec.setMaxClusterSize (15000);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);
	ec.setInputCloud (ground_deleted_cloud);
	ec.extract (cluster_indices);
	std::cout<<"EuclideanClusterExtraction\n";


	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Eclusted_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Eclusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*ground_deleted_cloud, *Eclusted_cloud);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
					Eclusted_cloud->points[*pit].r = colors[j%12][0];
					Eclusted_cloud->points[*pit].g = colors[j%12][1];
					Eclusted_cloud->points[*pit].b = colors[j%12][2];
      }
      j++;
	}
	sensor_msgs::PointCloud2 edit_cloud5;
  pcl::toROSMsg (*Eclusted_cloud, edit_cloud5);
	edit_cloud5.header.frame_id="/zed_current_frame";
  pc_pub5.publish(edit_cloud5);

	



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
					if(convert_coordinate_xz_nxz(x_temp,z_temp,nx,nz)){
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
			
    }
  }
	cloud->width=points_size;//points_size;
	cloud->height=1;
  cloud->points.resize (cloud->width * cloud->height);
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int k=0;k<(int)dem_element[nz][nx].size();k++){
		    pcl::PointXYZ point;
				invert_coordinate_xz_nxz(nx,nz,point.y,point.x);
		    point.z = dem_element[nz][nx][k];
		    cloud->points[count++]=point;
			}
    }
  }

//	std::cout<<"points_size:"<<points_size<<"\n";
//	std::cout<<"finished push_back\n";
	std::cout<<"resized size,count:"<<cloud->width * cloud->height<<","<<count<<"\n";
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
	edit_cloud.header.frame_id="ZED_left_camera";//"/zed_current_frame";
//	edit_cloud.header.stamp=ros::Time::now();
//	edit_cloud.header.seq+=1;
  pc_pub.publish(edit_cloud);

//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");  
//  viewer.showCloud (cloud);//.makeShared()); 
//	while (!viewer.wasStopped ()) {  
//  }
	std::cout<<"published\n";
//	cloud->points.clear();
//	cloud->clear();
//	std::cout<<"cleared size:"<<cloud->width * cloud->height<<"\n";
//  cloud->width  = width;
//  cloud->height = height;
//  cloud->points.resize (cloud->width * cloud->height);

	
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
//		dtct_obj.set_DEM_map();
		dtct_obj.convet_image_to_pcl();
//		std::cout<<"set demmap\n";
//		dtct_obj.clustering_DEM_elements();
//		std::cout<<"now processing!\n";
//		dtct_obj.convert_dem_to_pcl(); 
//		dtct_obj.clear_dem_element();
		time_cls.set_time();
		std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	
	}

	return 0;
}


