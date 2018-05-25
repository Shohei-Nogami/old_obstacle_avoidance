#include"detect_objects_class.h"



detect_objects::detect_objects()
	:it_pub1(nh_pub1),it_pub2(nh_pub2),EXECUTED_CALLBACK(false),cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud2(new pcl::PointCloud<pcl::PointXYZ>),voxeled_cloud (new pcl::PointCloud<pcl::PointXYZ>),ground_deleted_cloud (new pcl::PointCloud<pcl::PointXYZ>),inliers (new pcl::PointIndices),coefficients(new pcl::ModelCoefficients),selfvoxel_cloud(new pcl::PointCloud<pcl::PointXYZ>)//,index_img(new std::vector<index_image>[map_size_nz][map_size_nx])//,Eclusted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	//reserve memory
	index_img=new std::vector<index_image>*[map_size_nz];
	for(int i=0;i<map_size_nz;i++){
		index_img[i]=new std::vector<index_image>[map_size_nx];
	}
	dem_element=new std::vector<double>*[map_size_nz];
	for(int i=0;i<map_size_nz;i++){
		dem_element[i]=new std::vector<double>[map_size_nx];
	}
	slice_cluster=new std::vector< std::vector<cv::Point2i> >[map_size_nz];
	slice_cluster_index=new std::vector<int>*[map_size_nz];
	for(int i=0;i<map_size_nz;i++){
		slice_cluster_index[i]=new std::vector<int>[map_size_nx];
	}
	slice_cluster_velocity_element=new std::vector< std::vector<pcl::PointXYZ> >[map_size_nz];
	slice_cluster_velocity=new std::vector<pcl::PointXYZ>[map_size_nz];

	index_schm=new index_schema*[height];
	for(int i=0;i<height;i++){
		index_schm[i]=new index_schema[width];
	}

	cur_index_vxl=new index_voxel*[height];
	for(int i=0;i<height;i++){
		cur_index_vxl[i]=new index_voxel[width];
	}
	pre_index_vxl=new index_voxel*[height];
	for(int i=0;i<height;i++){
		pre_index_vxl[i]=new index_voxel[width];
	}
	voxel_element=new std::vector<pcl::PointXYZ>**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		voxel_element[j]=new std::vector<pcl::PointXYZ>*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			voxel_element[j][i]=new std::vector<pcl::PointXYZ>[map_size_ny];
		}
	}
	/*
	voxel_point=new pcl::PointXYZ**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		voxel_point[j]=new pcl::PointXYZ*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			voxel_point[j][i]=new pcl::PointXYZ[map_size_ny];
		}
	}
	*/

	voxel_point=new Point3f1i**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		voxel_point[j]=new Point3f1i*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			voxel_point[j][i]=new Point3f1i[map_size_ny];
		}
	}
	index_points=new int**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		index_points[j]=new int*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			index_points[j][i]=new int[map_size_ny];
		}
	}
	//reserve memory 
	cur_clusted_index=new int**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		cur_clusted_index[j]=new int*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			cur_clusted_index[j][i]=new int[map_size_ny];
		}
	}
	pre_clusted_index=new int**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		pre_clusted_index[j]=new int*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			pre_clusted_index[j][i]=new int[map_size_ny];
		}
	}

	//publisher and subscriber
	pub1=it_pub1.advertise("detected_objects_image",1);//test string
	pub2=it_pub2.advertise("detected_objects_image2",1);//test string
	
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&detect_objects::image_callback,this);
	nh_optflw.setCallbackQueue(&queue_optflw);
	sub_optflw=nh_optflw.subscribe("objects_velocity",1,&detect_objects::opticalflow_callback,this);
	
	nh_matching.setCallbackQueue(&queue_matching);
	sub_matching=nh_matching.subscribe("cluster_matching_index",1,&detect_objects::matching_callback,this);
	
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
  pc_pub7 = nh_pubpcl7.advertise<sensor_msgs::PointCloud2>("clusted_cloud_by_dem", 1);
  pc_pub8 = nh_pubpcl8.advertise<sensor_msgs::PointCloud2>("selfvoxcel_cloud", 1);
  pc_pub9 = nh_pubpcl9.advertise<sensor_msgs::PointCloud2>("Eclusted_cloud_by_selfvoxel", 1); 
  
  
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
		delete[] slice_cluster_index[i];
	}
	delete[] slice_cluster_index;

	for(int i = 0; i < height; ++i ) {
		delete[] index_schm[i];
	}
	delete[] index_schm;
	for(int i = 0; i < height; ++i ) {
		delete[] cur_index_vxl[i];
	}
	delete[] cur_index_vxl;
	for(int i = 0; i < height; ++i ) {
		delete[] pre_index_vxl[i];
	}
	delete[] pre_index_vxl;

	for(int j=0;j<map_size_nz;j++){
		for(int i=0;i<map_size_nx;i++){
			delete[] voxel_element[j][i];
		}
		delete[] voxel_element[j];
	}
	delete[] voxel_element;

	for(int j=0;j<map_size_nz;j++){
		for(int i=0;i<map_size_nx;i++){
			delete[] voxel_point[j][i];
		}
		delete[] voxel_point[j];
	}
	delete[] voxel_point;
	//memory release
	for(int j=0;j<map_size_nz;j++){
		for(int i=0;i<map_size_nx;i++){
			delete[] cur_clusted_index[j][i];
		}
		delete[] cur_clusted_index[j];
	}
	delete[] cur_clusted_index;	
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
bool detect_objects::culc_voxel_nxzy(const float voxel_size_x,const float voxel_size_z,const float voxel_size_y,const float x,const float z,const float y,int& nx,int& nz,int& ny){
	nx = (int)(2*x/voxel_size_x/2) + (int)(2*x/voxel_size_x) % 2;
	nz = (int)(2*z/voxel_size_z/2) + (int)(2*z/voxel_size_z) % 2;
	ny = (int)(y/voxel_size_y);
	if(map_size_nx/2<std::abs(nx)||map_size_nz<nz||ny<0||ny>map_size_ny){
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
bool detect_objects::convert_xzy_nxzy(const float& x,const float& z,const float& y,int& nx,int& nz,int& ny){
	nx = (int)(2*x/voxel_size_x/2) + (int)(2*x/voxel_size_x) % 2;
	nz = (int)(2*z/voxel_size_z/2) + (int)(2*z/voxel_size_z) % 2;
	ny = (int)(y/voxel_size_y);	
	if(map_size_nx/2<std::abs(nx)||map_size_nz<nz||map_size_ny<ny){
		return false;
	}
	else{
		nx+=map_size_nx/2;
		nz=map_size_nz-nz;
		return true;
	}
	
	
}

float detect_objects::culclate_euclid_distance(pcl::PointXYZ& p1,pcl::PointXYZ& p2){
		return(std::sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z) ) ) ;
}

float detect_objects::culclate_chebyshev_distance(pcl::PointXYZ& p1,pcl::PointXYZ& p2){

	if((p1.x-p2.x)>(p1.y-p2.y)){
		return ( (p1.x-p2.x) > (p1.z-p2.z) ? (p1.x-p2.x) : (p1.z-p2.z) );
	}
	else{	
		return( (p1.y-p2.y) > (p1.z-p2.z) ? (p1.y-p2.y) : (p1.z-p2.z) );
	}
}
float detect_objects::culclate_chebyshev_distance(Point3f1i& p1,Point3f1i& p2){

	if((p1.x-p2.x)>(p1.y-p2.y)){
		return ( (p1.x-p2.x) > (p1.z-p2.z) ? (p1.x-p2.x) : (p1.z-p2.z) );
	}
	else{	
		return( (p1.y-p2.y) > (p1.z-p2.z) ? (p1.y-p2.y) : (p1.z-p2.z) );
	}
}
float detect_objects::culclate_chebyshev_distance(pcl::PointXYZ& p1,Point3f1i& p2){

	if((p1.x-p2.x)>(p1.y-p2.y)){
		return ( (p1.x-p2.x) > (p1.z-p2.z) ? (p1.x-p2.x) : (p1.z-p2.z) );
	}
	else{	
		return( (p1.y-p2.y) > (p1.z-p2.z) ? (p1.y-p2.y) : (p1.z-p2.z) );
	}
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
/*
  a=coefficients->values[0];
  b=coefficients->values[1];
	c=coefficients->values[2];
	d=coefficients->values[3];
	sensor_msgs::PointCloud2 edit_cloud6;
  pcl::toROSMsg (*ground_points, edit_cloud6);
	edit_cloud6.header.frame_id="/zed_current_frame";
  pc_pub6.publish(edit_cloud6);
*/
}


void detect_objects::tracking_cluster(const std::vector<optical_flow_data>& opf_data,const float v,const float omg,const double& dt){//w：反時計周り
	int h,w;
	int nx,nz,ny;
	int scn;
	pcl::PointXYZ vX_opf;
	pcl::PointXYZ sum_vX_opf;
	pcl::PointXYZ ave_vX_opf;
	pcl::PointXYZ vX_odm;

	vX_odm.x=v*sin(-omg);
	vX_odm.y=0;
	vX_odm.z=v*cos(omg);

	for(int nz=0;nz<map_size_nz;nz++){
		slice_cluster_velocity_element[nz].resize( (int)slice_cluster[nz].size() );
		slice_cluster_velocity[nz].resize( (int)slice_cluster[nz].size() );
	}
	for(int n=0;n<opf_data.size();n++){
		h=opf_data[n].h;
		w=opf_data[n].w;
		nx=index_schm[h][w].nx;
		nz=index_schm[h][w].nz;
		ny=index_schm[h][w].ny;
		scn=slice_cluster_index[nz][nx][ny];
		vX_opf.x=opf_data[n].vx;
		vX_opf.y=opf_data[n].vy;
		vX_opf.z=opf_data[n].vz;
		slice_cluster_velocity_element[nz][scn].push_back(vX_opf);
	}
	for(int nz=0;nz<map_size_nz;nz++){
		for(int k=0;k<slice_cluster_velocity[nz].size();k++){
			for(int n=0;n<slice_cluster_velocity_element[nz][k].size();n++){
				sum_vX_opf.x=sum_vX_opf.x+slice_cluster_velocity_element[nz][k][n].x;
				sum_vX_opf.y=sum_vX_opf.y+slice_cluster_velocity_element[nz][k][n].y;
				sum_vX_opf.z=sum_vX_opf.z+slice_cluster_velocity_element[nz][k][n].z;
			}
			ave_vX_opf.x=sum_vX_opf.x/(int)slice_cluster_velocity_element[nz][k].size();
			ave_vX_opf.y=sum_vX_opf.y/(int)slice_cluster_velocity_element[nz][k].size();
			ave_vX_opf.z=sum_vX_opf.z/(int)slice_cluster_velocity_element[nz][k].size();
			slice_cluster_velocity[nz][k].x=ave_vX_opf.x+vX_odm.x;
			slice_cluster_velocity[nz][k].y=ave_vX_opf.y+vX_odm.y;
			slice_cluster_velocity[nz][k].z=ave_vX_opf.z+vX_odm.z;
		}
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

		std::cout<<"process_start:"<<time_cls.get_time_now()<<"\n";

    img_cls.set_image();

		std::cout<<"2:set_image:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.subscribe_depth_image();

		if(!dtct_obj.is_cvbridge_image()||!img_cls.is_cur_image())
			continue;

		std::cout<<"3:subscribe_depth_image:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.subscribe_opticalflow();

		std::cout<<"3.5:subscribe_opticalflow:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.set_depth_image();

		std::cout<<"4:set_depth_image:"<<time_cls.get_time_now()<<"\n";

		std::cout<<"5:clustering_start:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.create_voxel_grid(img_cls.get_cur_image_by_ref());

		std::cout<<"6:create_voxel_grid:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.voxel_filter();

		std::cout<<"7:voxel_filter:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.clusterig_selfvoxel();

		std::cout<<"8:clusterig_selfvoxel:"<<time_cls.get_time_now()<<"\n";

		if(dtct_obj.add_velocity_to_cluster())
		{
		
			std::cout<<"9:add_velocity_to_cluster:"<<time_cls.get_time_now()<<"\n";
		
			dtct_obj.estimate_velocity_of_cluster();

			std::cout<<"10:estimate_velocity_of_cluster:"<<time_cls.get_time_now()<<"\n";

			dtct_obj.draw_velocity(img_cls.get_cur_image_by_ref());

			std::cout<<"10:draw_velocity:"<<time_cls.get_time_now()<<"\n";
		}
/*
	dtct_obj.density_based_clustering(img_cls.get_cur_image_by_ref());

	std::cout<<"9:clusterig_by_density_based:"<<time_cls.get_time_now()<<"\n";

	img_cls.publish_debug_image( dtct_obj.draw_cluster(img_cls.get_cur_image_by_ref() ) );
*/

		//dtct_obj.tracking_cluster(/*hogehoge*/);


//		dtct_obj.convert_dem_to_pcl(); 
//		dtct_obj.publish_slice_cluster();
//		dtct_obj.clear_dem_element();
		time_cls.set_time();
		std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";

	}

	return 0;
}

