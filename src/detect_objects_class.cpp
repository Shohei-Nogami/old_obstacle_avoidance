#include"detect_objects_class.h"
#include"time_class.h"
#include"image_class.h"


detect_objects::detect_objects()
	:it_pub1(nh_pub1),it_pub2(nh_pub2),EXECUTED_CALLBACK(false),cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud2(new pcl::PointCloud<pcl::PointXYZ>),voxeled_cloud (new pcl::PointCloud<pcl::PointXYZ>),ground_deleted_cloud (new pcl::PointCloud<pcl::PointXYZ>),inliers (new pcl::PointIndices),coefficients(new pcl::ModelCoefficients),selfvoxel_cloud(new pcl::PointCloud<pcl::PointXYZ>)//,index_img(new std::vector<index_image>[map_size_nz][map_size_nx])//,Eclusted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
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
	
	index_vxl=new index_voxel*[height];
	for(int i=0;i<height;i++){
		index_vxl[i]=new index_voxel[width];
	}
	voxel_element=new std::vector<pcl::PointXYZ>**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		voxel_element[j]=new std::vector<pcl::PointXYZ>*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			voxel_element[j][i]=new std::vector<pcl::PointXYZ>[map_size_ny];
		}
	}
	voxel_point=new pcl::PointXYZ**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		voxel_point[j]=new pcl::PointXYZ*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			voxel_point[j][i]=new pcl::PointXYZ[map_size_ny];
		}
	}
	index_points=new int**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		index_points[j]=new int*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			index_points[j][i]=new int[map_size_ny];
		}
	}
	//publisher and subscriber
	pub1=it_pub1.advertise("detected_objects_image",1);//test string
	pub2=it_pub2.advertise("detected_objects_image2",1);//test string
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&detect_objects::image_callback,this);
	nh_optflw.setCallbackQueue(&queue_optflw);
	sub_optflw=nh_optflw.subscribe("objects_velocity",1,&detect_objects::opticalflow_callback,this);
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
		delete[] index_vxl[i];
	}
	delete[] index_vxl;
	
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
			delete[] clusted_index[j][i];
		}
		delete[] clusted_index[j];
	}
	delete[] clusted_index;	
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
void detect_objects::create_voxel_grid(cv::Mat& image){

	//const int map_size_nz=201;  //map height [pixcel]
	//const int map_size_nx=201;  //map width  [pixcel]
	//const int map_size_ny=38;  //map width  [pixcel]
	//const float ground_threshold=0.10;
	float floor_th=0.30+0.1;
	float camera_height=0.4125;
	float camera_bias=0.2;
	const float y_th=0.40;
	const float cam_y=0.4125;
	float z_temp,x_temp,y_temp;
	int cam_nx=map_size_nx/2;
	int cam_nz=map_size_nz-1;
	int nx,nz,ny;
	float a,b,c,d;
	std::cout<<"map_size_nz,nx,ny:("<<map_size_nz<<","<<map_size_nx<<","<<map_size_ny<<")\n";
	pcl::PointXYZ voxel_element_temp;
	
	//ground estimate
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
	//set voxel elements
	int original_size=0;
	
	//clear voxel_element
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				voxel_element[nz][nx][ny].clear();
			}
		}
	}
	float voxel_size_x=0.04;
	float voxel_size_z=0.08;
	float voxel_size_y=0.04;
	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			z_temp=depth_image.at<float>(h,w);
			if(z_temp>0.5&&!std::isinf(z_temp)){//
				x_temp=(w-width/2)*z_temp/f;
				y_temp=(height/2-h)*z_temp/f;
				y_ground=(-a*x_temp-b*y_temp-d)/c;
				//std::cout<<"y_t,y_g:"<<y_temp<<","<<y_ground<<"\n";
				//std::cout<<"y_temp+camera_height-y_ground:"<<y_temp+camera_height<<"\n";
			  if(y_temp-y_ground>0&&!std::isinf(y_temp)&&y_temp+camera_height<1.5
					&&(image.at<cv::Vec3b>(h,w)[0]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[1]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[2]<=255-color_th)){//){
					
					if(culc_voxel_nxzy(voxel_size_x,voxel_size_z,voxel_size_y,x_temp,z_temp,y_temp+camera_height,nx,nz,ny)){
						index_vxl[h][w].nz=nz;
						index_vxl[h][w].nx=nx;
						index_vxl[h][w].ny=ny;
						voxel_element_temp.x=x_temp;
						voxel_element_temp.y=y_temp;
						voxel_element_temp.z=z_temp;
						//std::cout<<"voxel_element_temp.y:"<<voxel_element_temp.y<<"\n";
						voxel_element[nz][nx][ny].push_back(voxel_element_temp);
						original_size++;
						//std::cout<<"index_img["<<nz<<"]["<<nx<<"].size:"<<index_img[nz][nx].size()<<"\n";
						continue;
					}
				}
			}
			index_vxl[h][w].nz=-1;
			index_vxl[h][w].nx=-1;
			index_vxl[h][w].ny=-1;
		}
	}
	std::cout<<"selfvoxel: original_size:"<<original_size<<"\n";
	//
	/*
	//show voxel area program 
		int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	test_points->points.clear();
	test_points->height=1;
	test_points->width=original_size;
	test_points->points.reserve(original_size);
	pcl::PointXYZRGB temp;
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				temp.r=colors[j%12][0];
				temp.g=colors[j%12][1];
				temp.b=colors[j%12][2];
				for(int k=0;k<voxel_element[nz][nx][ny].size();k++){
					temp.x=voxel_element[nz][nx][ny][k].z;
					temp.y=-voxel_element[nz][nx][ny][k].x;
					temp.z=voxel_element[nz][nx][ny][k].y;
					test_points->points.push_back(temp);
					
				}
				j++;
			}
		}
	}
	sensor_msgs::PointCloud2 edit_cloud6;
  pcl::toROSMsg (*test_points, edit_cloud6);
	edit_cloud6.header.frame_id="/zed_current_frame";
  pc_pub6.publish(edit_cloud6);
	*/
	
	//voxel process
	pcl::PointXYZ cog;//Center of gravity
	//float densy=0;
	//float cell_volume=cell_size*cell_size*cell_size_y;
	int voxel_size_threshold;//=3;
	int voxel_size=0;
	float dist;
	std::vector<index_voxel> clst_tsk;
	index_voxel clst_tsk_element;
	clst_tsk.reserve(original_size);
	/*
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				voxel_element[nz][nx][ny]=-1;
			}
		}
	}
	*/
	//int count_if=0;
	//int count_else=0;
	//int count_all=0;
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				//init Center of gravity
				cog.x=-1;
				cog.y=-1;
				cog.z=-1;
				//sum process
				dist=((float)map_size_nz-(float)nz)*voxel_size_z;
				if(dist>=2){
					voxel_size_threshold=dist*(-0.5)+5;
				}
				else{
					voxel_size_threshold=5;
				}
				if(voxel_size_threshold<0){
					voxel_size_threshold=0;
				}
				//voxel_size_threshold=3;
				if((int)voxel_element[nz][nx][ny].size()>voxel_size_threshold){
					//count_if++;
					cog.x=0;
					cog.y=0;
					cog.z=0;
					for(int k=0;k<voxel_element[nz][nx][ny].size();k++){
						//cog.x+=voxel_element[nz][nx][ny][k].x;
						//cog.y+=voxel_element[nz][nx][ny][k].y;
						//cog.z+=voxel_element[nz][nx][ny][k].z;
						cog.x+=voxel_element[nz][nx][ny][k].z;
						cog.y+=-voxel_element[nz][nx][ny][k].x;
						cog.z+=voxel_element[nz][nx][ny][k].y;
					}          
					cog.x/=(int)voxel_element[nz][nx][ny].size();
					cog.y/=(int)voxel_element[nz][nx][ny].size();
					cog.z/=(int)voxel_element[nz][nx][ny].size();
					voxel_point[nz][nx][ny]=cog;
					voxel_size++;
					
					clst_tsk_element.nx=nx;
					clst_tsk_element.nz=nz;
					clst_tsk_element.ny=ny;
					clst_tsk.push_back(clst_tsk_element);
					
				}
				else{
					//count_else++;
					voxel_point[nz][nx][ny]=cog;
				}
				//count_all++;
			}
		}
	}
	//std::cout<<"count;(if,else,all):("<<count_if<<","<<count_else<<","<<count_all<<")\n";
	std::cout<<"voxel_size:"<<voxel_size<<"\n";
	//convert voxel[][][] to pointcloud and write index
	selfvoxel_cloud->points.clear();
	selfvoxel_cloud->height=1;
	selfvoxel_cloud->width=voxel_size;
	selfvoxel_cloud->points.reserve(voxel_size);
	std::cout<<"reserved\n";
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				if(voxel_point[nz][nx][ny].x!=-1){
					/*std::cout<<voxel_point[nz][nx][ny].x<<","
										<<voxel_point[nz][nx][ny].y<<","
										<<voxel_point[nz][nx][ny].z<<"\n";
					*/
					index_points[nz][nx][ny]=(int)selfvoxel_cloud->points.size();
					selfvoxel_cloud->points.push_back(voxel_point[nz][nx][ny]);
				}
			}
		}
	}

	sensor_msgs::PointCloud2 edit_cloud;
	pcl::toROSMsg (*selfvoxel_cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
	pc_pub8.publish(edit_cloud);
	std::cout<<"finish_func\n";
	
	//euclid clustering
	//なぜかEuclidClusteringが使えない
	/*
	float clust_size=0.050f;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//何か探索用にツリーを作る
	tree->setInputCloud (selfvoxel_cloud);
	std::cout<<"1\n";
	std::vector<pcl::PointIndices> cluster_indices;
	std::cout<<"1\n";
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	std::cout<<"1\n";
	ec.setClusterTolerance (0.048);//同じクラスタとみなす距離
	std::cout<<"1\n";
	ec.setMinClusterSize (50);//クラスタを構成する最小の点数
	std::cout<<"2\n";
	ec.setMaxClusterSize (25000);//クラスタを構成する最大の点数
	std::cout<<"2\n";
	ec.setSearchMethod (tree);
	std::cout<<"2\n";
	ec.setInputCloud (selfvoxel_cloud);
	std::cout<<"2\n";
	ec.extract (cluster_indices);
	std::cout<<"3\n";
	std::cout<<"EuclideanClusterExtraction_by_selfvoxel\n";
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Eclusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*selfvoxel_cloud, *Eclusted_cloud);

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
	*/
	//clustering 

	std::vector<pcl::PointXYZ> cluster_elements;
	std::vector<cv::Point3i> cluster_elements_num;
	cv::Point3i cen_temp;
	//reserve memory 
	clusted_index=new int**[map_size_nz];
	for(int j=0;j<map_size_nz;j++){
		clusted_index[j]=new int*[map_size_nx];
		for(int i=0;i<map_size_nx;i++){
			clusted_index[j][i]=new int[map_size_ny];
		}
	}
	cluster.reserve(voxel_size);
	cluster_elements.reserve(voxel_size);
	cluster_elements_num.reserve(voxel_size);
	
	//init clusted_index
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				clusted_index[nz][nx][ny]=-1;
			}
		}
	}
	
	cluster.clear();
	
	//clustering process
	float clustering_distance=0.04*1.2;
	int serch_range_x=(int)(clustering_distance/voxel_size_x)+1;
	int serch_range_y=(int)(clustering_distance/voxel_size_y)+1;
	int serch_range_z=(int)(clustering_distance/voxel_size_z)+1;
	float edis;
	int cluster_size=0;
	std::cout<<"clustering process\n";
	
/*
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
*/
	for(int tsk_n=0;tsk_n<clst_tsk.size();tsk_n++){
		nz=clst_tsk[tsk_n].nz;
		nx=clst_tsk[tsk_n].nx;
		ny=clst_tsk[tsk_n].ny;
				if(voxel_point[nz][nx][ny].x==-1||clusted_index[nz][nx][ny]!=-1){//already clusted or point nothing
					//std::cout<<"("<<nz<<","<<nx<<","<<ny<<")\n";
					continue;//skip
				}
				
				/*
				std::cout<<voxel_point[nz][nx][ny].x<<","
									<<voxel_point[nz][nx][ny].y<<","
									<<voxel_point[nz][nx][ny].z<<"\n";
				*/
				cluster_elements.push_back(voxel_point[nz][nx][ny]);
				cen_temp.z=nz;
				cen_temp.x=nx;
				cen_temp.y=ny;
				cluster_elements_num.push_back(cen_temp);
				clusted_index[nz][nx][ny]=(int)cluster.size();
				//std::cout<<"nz,nx,ny):("<<nz<<","<<nx<<","<<ny<<")\n";
				for(int k=0;k<cluster_elements.size();k++){
					int cen_nz=cluster_elements_num[k].z;
					int cen_nx=cluster_elements_num[k].x;
					int cen_ny=cluster_elements_num[k].y;
					
					//std::cout<<"1\n";
					for(int srz=-serch_range_z+cen_nz;srz<serch_range_x+cen_nz;srz++){
						//std::cout<<"1\n";
						if(srz<0||srz>=map_size_nz){//outrange of array z
							continue;
						}
						for(int srx=-serch_range_x+cen_nx;srx<serch_range_x+cen_nx;srx++){
								//std::cout<<"1\n";
							if(srx<0||srx>=map_size_nx){//outrange of array x
								continue;
							}
							for(int sry=-serch_range_y+cen_ny;sry<serch_range_y+cen_ny;sry++){
								//std::cout<<"1\n";
								if(sry<0||sry>=map_size_ny||voxel_point[srz][srx][sry].x==-1
									||clusted_index[srz][srx][sry]!=-1){//outrange of array y or point nothing
									continue;
								}
								//Euclid distance
								//edis=culclate_euclid_distance(cluster_elements[k],voxel_point[srz][srx][sry]);
								//Chebyshev distance
								edis=culclate_chebyshev_distance(cluster_elements[k],voxel_point[srz][srx][sry]);
								/*
								std::cout<<cluster_elements[k].x<<","<<voxel_point[srz][srx][sry].x<<","
									<<cluster_elements[k].y<<","<<voxel_point[srz][srx][sry].y<<","
									<<cluster_elements[k].z<<","<<voxel_point[srz][srx][sry].z<<"\n";
								*/
								//edis=std::sqrt( (cluster_elements[k].x-voxel_point[srz][srx][sry].x)*(cluster_elements[k].x-voxel_point[srz][srx][sry].x) + (cluster_elements[k].y-voxel_point[srz][srx][sry].y)*(cluster_elements[k].y-voxel_point[srz][srx][sry].y) + (cluster_elements[k].z-voxel_point[srz][srx][sry].z)*(cluster_elements[k].z-voxel_point[srz][srx][sry].z) );
								//std::cout<<"edis:"<<edis<<"\n";
								if(edis<=clustering_distance){//clusted
									cluster_elements.push_back(voxel_point[srz][srx][sry]);
									cen_temp.z=srz;
									cen_temp.x=srx;
									cen_temp.y=sry;
									cluster_elements_num.push_back(cen_temp);
									clusted_index[srz][srx][sry]=(int)cluster.size();
									cluster_size++;
								}
							}//end for search range y
						}//end for search range x
					}//end for search range z
				}//end for k
				cluster.push_back(cluster_elements);
				cluster_elements.clear();
				cluster_elements_num.clear();
	}
	/*
			}//end for ny
		}//end for nx
	}//end for nz
	*/
	std::cout<<"cluster_size:"<<cluster_size<<"\n";
	//draw clusters
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB temp_point;
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト	
	
	//reserve clusted_cloud
	clusted_cloud->points.reserve(voxel_size);//cluster_size);
	clusted_cloud->height=1;
	clusted_cloud->width=voxel_size;	
	
	float volume_threshold=0.1*0.1*0.1;
	float one_point_volume=voxel_size_x*voxel_size_z*voxel_size_y;
	for(int cn=0;cn<cluster.size();cn++){
		if(one_point_volume*(int)cluster[cn].size()>volume_threshold){
			for(int cen=0;cen<cluster[cn].size();cen++){
				temp_point.x=cluster[cn][cen].x;
				temp_point.y=cluster[cn][cen].y;
				temp_point.z=cluster[cn][cen].z;
				temp_point.r=colors[j%12][0];
				temp_point.g=colors[j%12][1];
				temp_point.b=colors[j%12][2];
			
				clusted_cloud->points.push_back(temp_point);
			}
			j++;
		}
		else{
			for(int cen=0;cen<cluster[cn].size();cen++){
				temp_point.x=cluster[cn][cen].x;
				temp_point.y=cluster[cn][cen].y;
				temp_point.z=cluster[cn][cen].z;
				temp_point.r=0;
				temp_point.g=0;
				temp_point.b=0;
		
				clusted_cloud->points.push_back(temp_point);			
			}		
		}
	}
	std::cout<<"clusted_cloud->points:"<<clusted_cloud->points.size()<<"\n";
	std::cout<<"clusted_cloud->width:"<<clusted_cloud->width<<"\n";
	
	

	
	//publish point cloud
	sensor_msgs::PointCloud2 edit_cloud2;
  pcl::toROSMsg (*clusted_cloud, edit_cloud2);
	edit_cloud2.header.frame_id="/zed_current_frame";
  pc_pub9.publish(edit_cloud2);
	
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
void detect_objects::subscribe_opticalflow(void){
	queue_optflw.callOne(ros::WallDuration(1));
}
void detect_objects::opticalflow_callback(const obst_avoid::vel3d::ConstPtr& msg){
	vX.pt=msg->pt;
	vX.vel=msg->vel;
}
void detect_objects::add_velocity_to_cluster(void){
	std::vector< std::vector<pcl::PointXYZ> > cluster_vel;
	std::vector<pcl::PointXYZ> cluster_vel_element;
	pcl::PointXYZ vel_element;
	cluster_vel.resize(cluster.size());
	int h,w;
	int nx,nz,ny;
	int cn;
	for(int k=0;k<vX.pt.size();k++){
		h=vX.pt[k].h;
		w=vX.pt[k].w;
		nx=index_vxl[h][w].nx;
		ny=index_vxl[h][w].ny;
		nz=index_vxl[h][w].nz;
		cn=clusted_index[nx][nz][nz];
		if(cn){//cn!=-1
			vel_element.x=vX.vel[k].x;
			vel_element.y=vX.vel[k].y;
			vel_element.z=vX.vel[k].z;
			
			cluster_vel[cn].push_back(vel_element);
		}
		
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
				x_temp=(w-width/2)*z_temp/f;
				y_ground=(-a*x_temp-b*y_temp-d)/c;
//				std::cout<<"y_t,y_g:"<<y_temp<<","<<y_ground<<"\n";
			  if(y_temp>y_ground&&!std::isinf(y_temp)&&y_temp+camera_height<1.5
					&&(image.at<cv::Vec3b>(h,w)[0]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[1]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[2]<=255-color_th)){//){
					
					
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

	int dem_elements_size=0;
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			//dem_cluster[nz][nx].clear();
		  //copy y of index_image to dem_element
		  dem_elements_size+=(int)index_img[nz][nx].size();
		  dem_element[nz][nx].resize((int)index_img[nz][nx].size() );
		  for(int k=0;k<index_img[nz][nx].size();k++){
		    dem_element[nz][nx][k]=index_img[nz][nx][k].y;
		  }
		  /*
		  if(index_img[nz][nx].size()){
				std::cout<<"before:dem_element[nz][nx]:(";
				for(int k=0;k<index_img[nz][nx].size();k++){
				std::cout<<dem_element[nz][nx][k]<<",";
				}
				std::cout<<"\n";
			}
			*/
			std::sort(dem_element[nz][nx].begin(),dem_element[nz][nx].end());	
		  /*
		  if(index_img[nz][nx].size()){
				std::cout<<"after:dem_element[nz][nx]:(";
				for(int k=0;k<index_img[nz][nx].size();k++){
				std::cout<<dem_element[nz][nx][k]<<",";
				}
				std::cout<<"\n";
			}
			*/
		}
	}
	std::cout<<"dem_elements_size(before):"<<dem_elements_size<<"\n";
//	std::cout<<"did sort\n";
	dem_elements_size=0;
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			int iy_low=0;
			int iy_high=0;
			/*
			if(dem_element[nz][nx].size()){
				std::cout<<"dem_element["<<nz<<"]["<<nx<<"].size():"<<dem_element[nz][nx].size()<<"\n";
				//std::cout<<"iy:(low,high):\n";
			}
			*/
			for(int i=0;i<(int)dem_element[nz][nx].size()-1;i++){
				//std::cout<<"("<<iy_low<<","<<iy_high<<")\n";
				if(dem_element[nz][nx][i+1]-dem_element[nz][nx][i]<h_th){
					iy_high++;
					if(i==dem_element[nz][nx].size()-1-1){
						cv::Point2f y_low_high;
						y_low_high.x=dem_element[nz][nx][iy_low];//low
						y_low_high.y=dem_element[nz][nx][iy_high];//high
						dem_cluster[nz][nx].push_back(y_low_high);
						
					}
				}
				else{
					//std::cout<<"dem_element(min,max):("<<dem_element[nz][nx][0]<<","<<dem_element[nz][nx][(int)dem_element[nz][nx].size()-1]<<")\n";
					cv::Point2f y_low_high;
					y_low_high.x=dem_element[nz][nx][iy_low];//low
					y_low_high.y=dem_element[nz][nx][iy_high];//high
					//std::cout<<dem_element[nz][nx][iy_low]<<","<<dem_element[nz][nx][iy_high]<<"\n";
					dem_cluster[nz][nx].push_back(y_low_high);

					iy_low=iy_high+1;
					iy_high=iy_high+1;
				}
				
			}
			/*
			if(dem_element[nz][nx].size()){
				std::cout<<"iy_high:"<<iy_high<<"\n";
				for(int i=0;i<(int)dem_cluster[nz][nx].size()-1;i++){
					std::cout<<"["<<i<<"]:(low,high),("<<dem_cluster[nz][nx][i].x<<","<<dem_cluster[nz][nx][i].y<<")\n";
				}
			}
			*/
		}
	}
	
	dem_elements_size=0;
	std::cout<<"/write index of schema cluster\n";
	//write index of schema cluster
	float y_low,y_high;
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			//if(dem_cluster[nz][nx].size()){
			//	std::cout<<"dem_cluster["<<nz<<"]["<<nx<<"].size():"<<dem_cluster[nz][nx].size()<<"\n";
			//}
			for(int k=0;k<dem_cluster[nz][nx].size();k++){
				y_low=dem_cluster[nz][nx][k].x;//dem_element[nz][nx][dem_cluster[nz][nx][k].x];
				y_high=dem_cluster[nz][nx][k].y;//dem_element[nz][nx][dem_cluster[nz][nx][k].y];
				/*
				if(index_img[nz][nx].size()){
					std::cout<<"index_img.size:"<<index_img[nz][nx].size()<<"\n";
					std::cout<<"(low,high):("<<y_low<<","<<y_high<<")\n";
				}
				*/
				for(int k1=0;k1<index_img[nz][nx].size();k1++){
					//std::cout<<"dem_cluster[nz][nx][k]:("<<dem_cluster[nz][nx][k].x<<","<<dem_cluster[nz][nx][k].y<<")\n";
					/*
					std::cout<<k1<<":"<<index_img[nz][nx][k1].y<<":(";
					if(y_low<=index_img[nz][nx][k1].y){
						std::cout<<"○,";
					}
					else{
						std::cout<<"✕,";
					}
					if(y_high>=index_img[nz][nx][k1].y){
						std::cout<<"○)\n";
					}
					else{
						std::cout<<"✕)\n";
					}
					*/
					if(y_low<=index_img[nz][nx][k1].y&&y_high>=index_img[nz][nx][k1].y){
						dem_elements_size+=1;
						//std::cout<<"nz,nx,k,k1,h,w:"<<nz<<","<<nx<<","<<k<<","<<k1<<","<<index_img[nz][nx][k].h<<","<<index_img[nz][nx][k].w<<"\n";
//						std::cout<<"nz,nx,k,k1,h,w:"<<nz<<","<<nx<<","<<k<<","<<k1<<","<<index_img[nz][nx][k].h<<","<<index_img[nz][nx][k].w<<"\n";
						index_schm[index_img[nz][nx][k1].h][index_img[nz][nx][k1].w].ny=k;
					}
				}
		  }
		}
	}
	std::cout<<"end/write index of schema cluster\n";
	std::cout<<"dem_elements_size(after):"<<dem_elements_size<<"\n";
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
			slice_cluster_index[nz][nx].reserve((int)dem_cluster[nz][nx].size());		
			max_slic_clst+=(int)dem_cluster[nz][nx].size();
			max_slic_elm+=(int)dem_cluster[nz][nx].size();
			for(int k=0;k<(int)dem_cluster[nz][nx].size();k++){
				slice_cluster_index[nz][nx].push_back(-1);
				
			}
		}
		slice_cluster[nz].reserve(max_slic_clst);
	}
	slice_cluster_element.reserve(max_slic_elm);
	//std::cout<<"!\n";
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int k=0;k<(int)dem_cluster[nz][nx].size();k++){
				if(slice_cluster_index[nz][nx][k]!=-1){
					continue;
				}
				//std::cout<<"nz,nx,k:"<<nz<<","<<nx<<","<<k<<"\n";
				tp.x=nx;
				tp.y=k;
				slice_cluster_element.push_back(tp);
				slice_cluster_index[nz][nx][k]=(int)slice_cluster[nz].size();//be careful to transform the struct
				
				for(int k0=0;nx<map_size_nx-1&&k0<(int)slice_cluster_element.size();k0++){
					if(!ros::ok())
						break;
					int x0=slice_cluster_element[k0].x;
					int kk=slice_cluster_element[k0].y;
					for(int k1=0;k1<(int)dem_cluster[nz][x0+1].size();k1++){
						if(dem_cluster[nz][x0][kk].x<=dem_cluster[nz][x0+1][k1].y
							&&dem_cluster[nz][x0][kk].y>=dem_cluster[nz][x0+1][k1].x){
							//std::cout<<"if touched:slice_cluster_index["<<nz<<"]["<<x0+1<<"]["<<k1<<"]-("<<slice_cluster_index[nz][x0+1][k1]<<")\n";
							if(slice_cluster_index[nz][x0+1][k1]==-1){//isn't clusted
								tp.x=x0+1;
								tp.y=k1;
								slice_cluster_element.push_back(tp);
								slice_cluster_index[nz][x0+1][k1]=(int)slice_cluster[nz].size();
							}
							else{//is clusted
								//std::cout<<"is clusted:("<<slice_cluster_index[nz][x0+1][k1]<<","<<(int)slice_cluster[nz].size()<<"\n"; 
								if(slice_cluster_index[nz][x0+1][k1]!=(int)slice_cluster[nz].size()){
									//tp.x=x0+1;
									//tp.y=k1;
									//slice_cluster[nz][ slice_cluster_index[nz][x0+1][k1] ].push_back(tp);
									//
									//std::cout<<"aa\n";
									cv::Point2i marge_index_temp;
									bool already_marge=false;
									for(int mcn=0;mcn<marge_index.size();mcn++){
										if(marge_index[mcn].x==(int)slice_cluster[nz].size()
											&& marge_index[mcn].y==slice_cluster_index[nz][x0+1][k1] )
										{
											already_marge=true;
										}
									}
									if(!already_marge){
										marge_index_temp.x=(int)slice_cluster[nz].size();
										marge_index_temp.y=slice_cluster_index[nz][x0+1][k1];
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
		//adjust slice cluster index
		for(int nx=0;nx<map_size_nx;nx++){
			for(int k=0;k<slice_cluster_index[nz][nx].size();k++){
				for(int in=0;in<marge_index.size();in++){
					if(marge_index[in].x==slice_cluster_index[nz][nx][k]){
						slice_cluster_index[nz][nx][k]=marge_index[in].y;
						break;
					}
				}
			}
		}
		//reset marge_index
		marge_index.clear();
		
	}//end for nz

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
				//if((int)dem_element[nz][nx].size()<=1){
				/*
				if((int)slice_cluster[nz][k].size()<=1){//cluster num filtering
					continue;
				}
				*/
				float cluster_size=cell_size*(h_high-h_low);
				const float cluster_size_threshold=0.04*0.04;
				if(cluster_size<cluster_size_threshold){
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
			slice_cluster_index[nz][nx].clear();
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
		dtct_obj.convet_image_to_pcl(img_cls.get_cur_image_by_ref());
//		std::cout<<"6:"<<time_cls.get_time_now()<<"\n";
		dtct_obj.clustering_DEM_elements();//<-

		std::cout<<"7:"<<time_cls.get_time_now()<<"\n";
//		std::cout<<"now processing!\n";
		dtct_obj.clustering_slice();
		std::cout<<"8:"<<time_cls.get_time_now()<<"\n";
		//dtct_obj.tracking_cluster(/*hogehoge*/);
		dtct_obj.create_voxel_grid(img_cls.get_cur_image_by_ref());
		
		
		dtct_obj.convert_dem_to_pcl(); 
		dtct_obj.publish_slice_cluster();
		dtct_obj.clear_dem_element();
		time_cls.set_time();
		std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	
	}

	return 0;
}

