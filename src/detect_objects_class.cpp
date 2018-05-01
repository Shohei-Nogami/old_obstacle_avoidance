#include"detect_objects_class.h"
#include"time_class.h"
#include"image_class.h"


detect_objects::detect_objects()
	:it_pub1(nh_pub1),it_pub2(nh_pub2),EXECUTED_CALLBACK(false),cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud2(new pcl::PointCloud<pcl::PointXYZ>),voxeled_cloud (new pcl::PointCloud<pcl::PointXYZ>),ground_deleted_cloud (new pcl::PointCloud<pcl::PointXYZ>),inliers (new pcl::PointIndices),coefficients(new pcl::ModelCoefficients)//,index_img(new std::vector<index_image>[map_size_nz][map_size_nx])//,Eclusted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
/*	testes=new float*[500];
	for(int i=0;i<500;i++){
		testes[i] =new float[299];
	}
	index_img=new index_image*[map_size_nz];
	for(int i=0;i<map_size_nz;i++){
		index_img[i]=new index_image[map_size_nx];
	}
*/
	
	index_img=new std::vector<index_image>*[map_size_nz];
	for(int i=0;i<map_size_nz;i++){
		index_img[i]=new std::vector<index_image>[map_size_nx];
	}
	dem_element=new std::vector<double>*[map_size_nz];
	for(int i=0;i<map_size_nz;i++){
		dem_element[i]=new std::vector<double>[map_size_nx];
	}
	slice_cluster=new std::vector< std::vector<cv::Point2i> >[map_size_nz];
	clusted_flag=new std::vector<int>*[map_size_nz];
	for(int i=0;i<map_size_nz;i++){
		clusted_flag[i]=new std::vector<int>[map_size_nx];
	}
	index_schm=new index_schema*[height];
	for(int i=0;i<height;i++){
		index_schm[i]=new index_schema[width];
	}

	pub1=it_pub1.advertise("detected_objects_image",1);//test string
	pub2=it_pub2.advertise("detected_objects_image2",1);//test string
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&detect_objects::image_callback,this);

	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			dem_element[nz][nx].reserve(height);
			dem_cluster[nz][nx].reserve(height);
			index_img[nz][nx].reserve(height);
		}
	}
  pc_pub = nh_pubpcl.advertise<sensor_msgs::PointCloud2>("dem_cloud", 1);
  pc_pub2 = nh_pubpcl2.advertise<sensor_msgs::PointCloud2>("trans_image_to_cloud", 1);
  pc_pub3 = nh_pubpcl3.advertise<sensor_msgs::PointCloud2>("vocel_cloud", 1);
  pc_pub4 = nh_pubpcl4.advertise<sensor_msgs::PointCloud2>("ground_delete_cloud", 1);
  pc_pub5 = nh_pubpcl5.advertise<sensor_msgs::PointCloud2>("Eclusted_cloud", 1);
  pc_pub6 = nh_pubpcl6.advertise<sensor_msgs::PointCloud2>("test_cloud", 1);
  pc_pub7 = nh_pubpcl7.advertise<sensor_msgs::PointCloud2>("Eclusted_cloud_by_dem", 1);
  
//  cloud->width  = width;
//  cloud->height = height;
//  cloud->points.resize (cloud->width * cloud->height);

//床面抽出
	seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出

	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (500);//RANSACの繰り返し回数
	seg.setDistanceThreshold (0.10);//モデルとどのくらい離れていてもいいか???謎
	seg.setAxis(Eigen::Vector3f (0.0,0.0,1.0));//法線ベクトル
	seg.setEpsAngle(15.0f * (M_PI/180.0f));//許容出来る平面

}
detect_objects::~detect_objects(){
//	delete[]
	for(int i = 0; i < map_size_nz; ++i ) {
		delete[] index_img[i];
	}
	delete[] index_img;
	
	for(int i = 0; i < map_size_nz; ++i ) {
		delete[] dem_element[i];
	}
	delete[] dem_element;
	
	delete[] slice_cluster;
	for(int i = 0; i < map_size_nz; ++i ) {
		delete[] clusted_flag[i];
	}
	delete[] clusted_flag;
	
	for(int i = 0; i < height; ++i ) {
		delete[] index_schm[i];
	}
	delete[] index_schm;
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
	if(map_size_nx/2<std::abs(nx)||map_size_nz<nz){
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
void detect_objects::convet_image_to_pcl(cv::Mat& image){
//	std::cout<<"1\n";
	cloud2->width  = width;
  cloud2->height = height;
	
  cloud2->points.resize (cloud2->width * cloud2->height);
	const float ground_threshold=0;	
	float z_temp,x_temp,y_temp;
	float cam_x=0.12/2;
	int k=0;
//	cv::Vec3b white(255,255,255);
	uint8_t color_th=3;
	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			z_temp=depth_image.at<float>(h,w);
			if(z_temp>0.5&&!std::isinf(z_temp)
				&&(image.at<cv::Vec3b>(h,w)[0]<=255-color_th
				||image.at<cv::Vec3b>(h,w)[1]<=255-color_th
				||image.at<cv::Vec3b>(h,w)[2]<=255-color_th)){//
				y_temp=((float)height/2-(float)h)*z_temp/f;
				if(y_temp+0.4125<1.5){
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

  sensor_msgs::PointCloud2 edit_cloud;
  pcl::toROSMsg (*cloud2, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
  pc_pub2.publish(edit_cloud);

	float voxel_size=0.050f;//=0.030f;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud2);
	vg.setLeafSize (voxel_size, voxel_size, voxel_size);
	vg.filter (*voxeled_cloud);
	std::cout<<"voxceled size:"<< voxeled_cloud->points.size()<<"\n";

  sensor_msgs::PointCloud2 edit_cloud3;
  pcl::toROSMsg (*voxeled_cloud, edit_cloud3);
	edit_cloud3.header.frame_id="/zed_current_frame";
  pc_pub3.publish(edit_cloud3);


	seg.setInputCloud (voxeled_cloud);
	seg.segment (*inliers, *coefficients);
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
/*	
	
	float floor_th=0.05;
	float camera_height=0.4125;
	float camera_bias=0;

	if(std::abs(coefficients->values[3]-(camera_height+camera_bias))>=0.15){
		coefficients->values[0]=-0.08;
		coefficients->values[1]=0;
		coefficients->values[2]=1;
		coefficients->values[3]=(camera_height+camera_bias)-floor_th;
	}
	else{
		coefficients->values[3]-=floor_th;
	}
 std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
*/
	float floor_th=0.30;
	float camera_height=0.4125;
	float camera_bias=0;
	const float y_th=0.40;
	const float cam_y=0.4125;
	float a,b,c,d;
	ground_estimation_from_pc(y_th,cam_y,a,b,c,d);
	
	if(std::abs(d-(camera_height+camera_bias))>=0.15){
		a=-0.08;
		b=0;
		c=1;
		d=(camera_height+camera_bias)-floor_th;
	}
	else{
		d-=floor_th;
	}


	ground_deleted_cloud->points.clear();
	ground_deleted_cloud->points.reserve(ground_deleted_cloud->width);
//	std::cout<<"d:"<<d<<"\n";
	for(int k=0;k<voxeled_cloud->points.size();k++){
//		y_temp=(-coefficients->values[0]*voxeled_cloud->points[k].x-coefficients->values[1]*voxeled_cloud->points[k].y-coefficients->values[3])/coefficients->values[2];
		y_temp=(-a*x_temp-b*y_temp-d)/c;
		if(y_temp<voxeled_cloud->points[k].z){
			ground_deleted_cloud->points.push_back(voxeled_cloud->points[k]);
		}
	}
	
	ground_deleted_cloud->width=ground_deleted_cloud->points.size();
  ground_deleted_cloud->height = 1;
	std::cout<<"ground_deleted_cloud->width:"<<ground_deleted_cloud->width<<"\n";
	

	sensor_msgs::PointCloud2 edit_cloud4;
  pcl::toROSMsg (*ground_deleted_cloud, edit_cloud4);
	edit_cloud4.header.frame_id="/zed_current_frame";
  pc_pub4.publish(edit_cloud4);

	std::cout<<"publish ground_deleted_cloud\n";
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//何か探索用にツリーを作る
	tree->setInputCloud (ground_deleted_cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (voxel_size*1.2);//同じクラスタとみなす距離
	ec.setMinClusterSize ((int)(0.02/voxel_size*100));//クラスタを構成する最小の点数
	ec.setMaxClusterSize (/*15000*/25000);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);
	ec.setInputCloud (ground_deleted_cloud);
	ec.extract (cluster_indices);
	std::cout<<"EuclideanClusterExtraction\n";


	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
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

void detect_objects::ground_estimation_from_image(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d){
  	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
  float x_temp;
  float y_temp;
  float z_temp;
  pcl::PointXYZ p_temp;
  //ground_points->points.clear();
  ground_points->points.reserve(height/2*width);
  for(int h=height/2+1;h<height;h++){
    for(int w=0;w<width;w++){
      z_temp=depth_image.at<float>(h,w);
      if(z_temp>0.5&&!std::isinf(z_temp)){
        y_temp=(height/2-h)*z_temp/f;
        if(std::abs(y_temp+cam_y)<y_th){
          x_temp=-( ((float)w-(float)width/2)*z_temp/f-cam_y );
          
          p_temp.x=z_temp;
          p_temp.y=x_temp;
          p_temp.z=y_temp;
          ground_points->points.push_back(p_temp);
        }
      }
    }
  }
  ground_points->width=ground_points->points.size();
  ground_points->height=1;

  seg.setInputCloud (ground_points);

	seg.segment (*inliers, *coefficients);
	std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
   a=coefficients->values[0];
   b=coefficients->values[1];
	c=coefficients->values[2];
	d=coefficients->values[3];  

}
void detect_objects::ground_estimation_from_pc(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d){
  	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);

  //ground_points->points.clear();
  ground_points->points.reserve(voxeled_cloud->points.size());
  for(int k=0;k<voxeled_cloud->points.size();k++){
//	   if(std::abs(voxeled_cloud->points[k].z+cam_y)<y_th){
	   if(voxeled_cloud->points[k].z+cam_y<y_th&&voxeled_cloud->points[k].z+cam_y>-0.05){

      ground_points->points.push_back(voxeled_cloud->points[k]);
    }
  }
  ground_points->width=ground_points->points.size();
  ground_points->height=1;

	seg.setInputCloud (ground_points);

	seg.segment (*inliers, *coefficients);
	std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
  a=coefficients->values[0];
  b=coefficients->values[1];
	c=coefficients->values[2];
	d=coefficients->values[3];
	sensor_msgs::PointCloud2 edit_cloud6;
  pcl::toROSMsg (*ground_points, edit_cloud6);
	edit_cloud6.header.frame_id="/zed_current_frame";
  pc_pub6.publish(edit_cloud6);
}


void detect_objects::set_DEM_map(cv::Mat& image){

	//const float ground_threshold=0.10;
	float floor_th=0.30;
	float camera_height=0.4125;
	float camera_bias=0.2;
	const float y_th=0.40;
	const float cam_y=0.4125;
	float z_temp,x_temp,y_temp;
	int cam_nx=map_size_nx/2;
	int cam_nz=map_size_nz-1;
	int nx,nz;
	float a,b,c,d;

	index_image index_img_temp;
	ground_estimation_from_image(y_th,cam_y,a,b,c,d);
	float y_ground;

	uint8_t color_th=3;
	
	if(std::abs(d-(camera_height+camera_bias))>=0.15){
		a=-0.08;
		b=0;
		c=1;
		d=(camera_height+camera_bias)-floor_th;
	}
	else{
		d-=floor_th;
	}
	std::cout<<a<<" x + "<<b<<" y + "<<c<<" z + "<<d<<" = 0\n";
//	std::cout<<"411\n";
	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			z_temp=depth_image.at<float>(h,w);
			if(z_temp>0.5&&!std::isinf(z_temp)){//
				y_temp=(height/2-h)*z_temp/f;
				y_ground=(-a*x_temp-b*y_temp-d)/c;
//				std::cout<<"y_t,y_g:"<<y_temp<<","<<y_ground<<"\n";
			  if(y_temp>y_ground&&!std::isinf(y_temp)&&y_temp<1.5
					&&(image.at<cv::Vec3b>(h,w)[0]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[1]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[2]<=255-color_th)){//){
					
					x_temp=(w-width/2)*z_temp/f;
					if(convert_coordinate_xz_nxz(x_temp,z_temp,nx,nz)){
						//dem_element[nz][nx].push_back(y_temp);
						index_schm[h][w].nz=nz;
						index_schm[h][w].nx=nx;
						index_schm[h][w].y=y_temp;
						index_img_temp.h=h;
						index_img_temp.w=w;
						index_img_temp.y=y_temp;
						index_img[nz][nx].push_back(index_img_temp);
						//std::cout<<"index_img["<<nz<<"]["<<nx<<"].size:"<<index_img[nz][nx].size()<<"\n";
						continue;
					}
				}
			}
			index_schm[h][w].nz=-1;
			index_schm[h][w].nx=-1;
		}
	}
}


void detect_objects::clustering_DEM_elements(void){


	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			//dem_cluster[nz][nx].clear();
		  //copy y of index_image to dem_element
		  dem_element[nz][nx].resize(index_img[nz][nx].size() );
		  for(int k=0;k<index_img[nz][nx].size();k++){
		    dem_element[nz][nx][k]=index_img[nz][nx][k].y;
		  }
			std::sort(dem_element[nz][nx].begin(),dem_element[nz][nx].end());
		}
	}
//	std::cout<<"did sort\n";
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			int iy_low=0;
			int iy_high=0;
			if(dem_element[nz][nx].size()){
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
	std::cout<<"/write index of schema cluster\n";
	//write index of schema cluster
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int k=0;k<dem_cluster[nz][nx].size();k++){
				for(int k1=0;k1<index_img[nz][nx].size();k1++){
					if(dem_cluster[nz][nx][k].x<=index_img[nz][nx][k1].y&&dem_cluster[nz][nx][k].y>=index_img[nz][nx][k1].y){
					//	std::cout<<"nz,nx,k,k1,h,w:"<<nz<<","<<nx<<","<<k<<","<<k1<<","<<index_img[nz][nx][k].h<<","<<index_img[nz][nx][k].w<<"\n";
						index_schm[index_img[nz][nx][k].h][index_img[nz][nx][k].w].ny=k;
//						std::cout<<"nz,nx,k,k1,h,w:"<<nz<<","<<nx<<","<<k<<","<<k1<<","<<index_img[nz][nx][k].h<<","<<index_img[nz][nx][k].w<<"\n";
						index_schm[index_img[nz][nx][k1].h][index_img[nz][nx][k1].w].ny=k;
					}
					else{
						break;
					}
				}
		  }
		}
	}
	std::cout<<"end/write index of schema cluster\n";
	
}

void detect_objects::clustering_slice(void){
	std::vector<cv::Point2i> slice_cluster_element;
	cv::Point2i tp;//x:i,y:k
	int max_slic_clst=0;
	int max_slic_elm=0;		
	//std::vector<int> marge_index;
	std::vector<cv::Point2i> marge_index;
	marge_index.reserve(100);
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			//std::cout<<"nz,nx,k:"<<nz<<","<<nx<<"\n";
			clusted_flag[nz][nx].reserve((int)dem_cluster[nz][nx].size());		
			max_slic_clst+=(int)dem_cluster[nz][nx].size();
			max_slic_elm+=(int)dem_cluster[nz][nx].size();
			for(int k=0;k<(int)dem_cluster[nz][nx].size();k++){
				clusted_flag[nz][nx].push_back(-1);
				
			}
		}
		slice_cluster[nz].reserve(max_slic_clst);
	}
	slice_cluster_element.reserve(max_slic_elm);
	//std::cout<<"!\n";
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int k=0;k<(int)dem_cluster[nz][nx].size();k++){
				if(clusted_flag[nz][nx][k]!=-1){
					continue;
				}
				//std::cout<<"nz,nx,k:"<<nz<<","<<nx<<","<<k<<"\n";
				tp.x=nx;
				tp.y=k;
				slice_cluster_element.push_back(tp);
				clusted_flag[nz][nx][k]=(int)slice_cluster[nz].size();//be careful to transform the struct
				
				for(int k0=0;nx<map_size_nx-1&&k0<(int)slice_cluster_element.size();k0++){
					if(!ros::ok())
						break;
					int x0=slice_cluster_element[k0].x;
					int kk=slice_cluster_element[k0].y;
					for(int k1=0;k1<(int)dem_cluster[nz][x0+1].size();k1++){
						if(dem_cluster[nz][x0][kk].x<=dem_cluster[nz][x0+1][k1].y
							&&dem_cluster[nz][x0][kk].y>=dem_cluster[nz][x0+1][k1].x){
							//std::cout<<"if touched:clusted_flag["<<nz<<"]["<<x0+1<<"]["<<k1<<"]-("<<clusted_flag[nz][x0+1][k1]<<")\n";
							if(clusted_flag[nz][x0+1][k1]==-1){//isn't clusted
								tp.x=x0+1;
								tp.y=k1;
								slice_cluster_element.push_back(tp);
								clusted_flag[nz][x0+1][k1]=(int)slice_cluster[nz].size();
							}
							else{//is clusted
								//std::cout<<"is clusted:("<<clusted_flag[nz][x0+1][k1]<<","<<(int)slice_cluster[nz].size()<<"\n";
								if(clusted_flag[nz][x0+1][k1]!=(int)slice_cluster[nz].size()){
									//tp.x=x0+1;
									//tp.y=k1;
									//slice_cluster[nz][ clusted_flag[nz][x0+1][k1] ].push_back(tp);
									//
									//std::cout<<"aa\n";
									cv::Point2i marge_index_temp;
									bool already_marge=false;
									for(int mcn=0;mcn<marge_index.size();mcn++){
										if(marge_index[mcn].y==clusted_flag[nz][x0+1][k1]){
											already_marge=true;
										}
									}
									if(!already_marge){
										marge_index_temp.x=(int)slice_cluster[nz].size();
										marge_index_temp.y=clusted_flag[nz][x0+1][k1];
										marge_index.push_back(marge_index_temp);
									}
								}
							}//end else
						}//end if touched
					}//end for k1
				}//end for k0

				slice_cluster[nz].push_back(slice_cluster_element);
				slice_cluster_element.clear();
				//std::cout<<"end for k\n";
			}//end for k
		}//end for nx

		for(int in=0;in<marge_index.size();in++){
			//clustering
			for(int k=0;k<slice_cluster[nz][marge_index[in].x].size();k++){
				slice_cluster[nz][marge_index[in].y].push_back(slice_cluster[nz][marge_index[in].x][k]);
			}
			//adjust index
			for(int inn=in+1;inn<marge_index.size();inn++){
				if(marge_index[in].x==marge_index[inn].y){
					marge_index[inn].y=marge_index[in].y;
				}
			}
		}
		//delete marged clusters
		//std::cout<<"delete marged clusters\n";
		for(int in=0;in<marge_index.size();in++){
			slice_cluster[nz].erase(slice_cluster[nz].begin()+marge_index[in].x);
		}
		
		//reset marge_index
		marge_index.clear();
		
	}//end for nz

}
void detect_objects::publish_slice_cluster(void){
	
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Eclusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::copyPointCloud(*ground_deleted_cloud, *Eclusted_cloud);
	pcl::PointXYZRGB cloud_element;
	int cloud_size=0;
	for(int nz=0;nz<map_size_nz;nz++){
		for(int k=0;k<slice_cluster[nz].size();k++){
			cloud_size+=(int)slice_cluster[nz].size();
		}
	}
	//std::cout<<"cloud_size:"<<cloud_size<<"\n";
	Eclusted_cloud->points.reserve(cloud_size);
	for(int nz=0;nz<map_size_nz;nz++){
		for(int k=0;k<slice_cluster[nz].size();k++){
			
			//if((int)slice_cluster[nz][k].size()<=1){
			//	continue;
			//}
			//set color
			cloud_element.r = colors[j%12][0];
			cloud_element.g = colors[j%12][1];
			cloud_element.b = colors[j%12][2];
			for(int k0=0;k0<slice_cluster[nz][k].size();k0++){

				//set x,y
				int nx=slice_cluster[nz][k][k0].x;
				invert_coordinate_xz_nxz(nx,nz,cloud_element.y,cloud_element.x);
				int dem_clst_k=slice_cluster[nz][k][k0].y;
				float h_low=dem_cluster[nz][nx][dem_clst_k].x;
				float h_high=dem_cluster[nz][nx][dem_clst_k].y;
				//std::cout<<"h_low,h_high:"<<h_low<<","<<h_high<<"\n";
				//search z
				int hn_low=0;
				int hn_high=0;
				//if((int)dem_element[nz][nx].size()<=1){
				if((int)slice_cluster[nz][k].size()<=1){
					continue;
				}
				for(int k1=0;k1<dem_element[nz][nx].size();k1++){
					
					if(h_low<=dem_element[nz][nx][k1]&&h_high>=dem_element[nz][nx][k1]){
						cloud_element.z=dem_element[nz][nx][k1];
						Eclusted_cloud->points.push_back(cloud_element);
					}
				}
			}			
			j++;
		}
	}
	Eclusted_cloud->height=1;
	Eclusted_cloud->width=(int)Eclusted_cloud->points.size();
	std::cout<<"Eclusted_cloud->points.size():"<<Eclusted_cloud->points.size()<<"\n";
	sensor_msgs::PointCloud2 edit_cloud;
  pcl::toROSMsg (*Eclusted_cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
  pc_pub7.publish(edit_cloud);
  
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

	std::cout<<"resized size,count:"<<cloud->width * cloud->height<<","<<count<<"\n";

  sensor_msgs::PointCloud2 edit_cloud;
  pcl::toROSMsg (*cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
  pc_pub.publish(edit_cloud);

	std::cout<<"published\n";
	
}
void detect_objects::clear_dem_element(void){
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			dem_element[nz][nx].clear();
			index_img[nz][nx].clear();
			clusted_flag[nz][nx].clear();
			dem_cluster[nz][nx].clear();
		}
		for(int k=0;k<slice_cluster[nz].size();k++){
			slice_cluster[nz][k].clear();
		}
		slice_cluster[nz].clear();
	}

}

int main(int argc,char **argv){
	ros::init(argc,argv,"detect_objects_class_test");
	detect_objects dtct_obj;
  time_class time_cls;
	image_class img_cls;
	img_cls.define_variable();
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
		std::cout<<"1:"<<time_cls.get_time_now()<<"\n";
    img_cls.set_image();
		std::cout<<"2:"<<time_cls.get_time_now()<<"\n";
		dtct_obj.subscribe_depth_image();
		if(!dtct_obj.is_cvbridge_image()||!img_cls.is_cur_image())
			continue;
		std::cout<<"3:"<<time_cls.get_time_now()<<"\n";
//		std::cout<<"subsctibed cvbridgeimage\n";
		dtct_obj.set_depth_image();
		std::cout<<"4:"<<time_cls.get_time_now()<<"\n";
//		std::cout<<"set matimage\n";
		dtct_obj.set_DEM_map(img_cls.get_cur_image_by_ref());
//		std::cout<<"5:"<<time_cls.get_time_now()<<"\n";
//		dtct_obj.convet_image_to_pcl(img_cls.get_cur_image_by_ref());
//		std::cout<<"6:"<<time_cls.get_time_now()<<"\n";
		dtct_obj.clustering_DEM_elements();//<-

		std::cout<<"7:"<<time_cls.get_time_now()<<"\n";
//		std::cout<<"now processing!\n";
		dtct_obj.clustering_slice();
		std::cout<<"8:"<<time_cls.get_time_now()<<"\n";
//		dtct_obj.convert_dem_to_pcl(); 
		dtct_obj.publish_slice_cluster();
		dtct_obj.clear_dem_element();
		time_cls.set_time();
		std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	
	}

	return 0;
}

