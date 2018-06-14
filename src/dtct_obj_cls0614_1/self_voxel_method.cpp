#include"detect_objects_class.h"



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
	//if(1){
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
				//voxel_element[nz][nx][ny].clear();
				voxel_point[nz][nx][ny].s=0;
				voxel_point[nz][nx][ny].x=0;
				voxel_point[nz][nx][ny].y=0;
				voxel_point[nz][nx][ny].z=0;

			}
		}
	}
	
	std::cout<<"00:"<<tm_cls.get_time_now()<<"\n";
	for(int h=0;h<height;h++){
		for(int w=0;w<width;w++){
			
			pre_index_vxl[h][w]=cur_index_vxl[h][w];
			
			z_temp=depth_image.at<float>(h,w);
			if(z_temp>0.5&&!std::isinf(z_temp)){//
				x_temp=(w-width/2)*z_temp/f;
				y_temp=(height/2-h)*z_temp/f;
				y_ground=(-a*z_temp-b*(-x_temp)-d)/c;
				//std::cout<<"y_t,y_g:"<<y_temp<<","<<y_ground<<"\n";
				//std::cout<<"y_temp+camera_height-y_ground:"<<y_temp+camera_height<<"\n";
			  if(y_temp-y_ground>0&&!std::isinf(y_temp)&&y_temp+camera_height<1.5
					&&(image.at<cv::Vec3b>(h,w)[0]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[1]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[2]<=255-color_th)){//){

					if(culc_voxel_nxzy(voxel_size_x,voxel_size_z,voxel_size_y,x_temp,z_temp,y_temp+camera_height,nx,nz,ny)){
						cur_index_vxl[h][w].nz=nz;
						cur_index_vxl[h][w].nx=nx;
						cur_index_vxl[h][w].ny=ny;
						//voxel_point[nz][nx][ny].x+=x_temp;
						//voxel_point[nz][nx][ny].y+=y_temp;
						//voxel_point[nz][nx][ny].z+=z_temp;
						voxel_point[nz][nx][ny].x+=z_temp;
						voxel_point[nz][nx][ny].y+=-x_temp;
						voxel_point[nz][nx][ny].z+=y_temp;
						voxel_point[nz][nx][ny].s++;

						original_size++;
						continue;
					}
				}
			}
			cur_index_vxl[h][w].nz=-1;
			//cur_index_vxl[h][w].nx=-1;
			//cur_index_vxl[h][w].ny=-1;
		}
	}
	std::cout<<"selfvoxel: original_size:"<<original_size<<"\n";
}

void detect_objects::voxel_filter(void){
	std::cout<<"voxel_filter_start:"<<tm_cls.get_time_now()<<"\n";

	//voxel process
	pcl::PointXYZ cog;//Center of gravity
	//float densy=0;
	//float cell_volume=cell_size*cell_size*cell_size_y;
	int voxel_size_threshold;//=3;
	voxel_size=0;
	float dist;
	//std::vector<index_voxel> clst_tsk;
	//index_voxel clst_tsk_element;
	//clst_tsk.reserve(original_size);

	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				//
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
				if((int)voxel_point[nz][nx][ny].s>voxel_size_threshold){
					//count_if++;
					//ave process
			
					voxel_point[nz][nx][ny].x=voxel_point[nz][nx][ny].x/voxel_point[nz][nx][ny].s;
					voxel_point[nz][nx][ny].y=voxel_point[nz][nx][ny].y/voxel_point[nz][nx][ny].s;
					voxel_point[nz][nx][ny].z=voxel_point[nz][nx][ny].z/voxel_point[nz][nx][ny].s;
					
					voxel_size++;

					//clst_tsk_element.nx=nx;
					//clst_tsk_element.nz=nz;
					//clst_tsk_element.ny=ny;
					//clst_tsk.push_back(clst_tsk_element);

				}
				else{
					//count_else++;
					voxel_point[nz][nx][ny].s=0;
				}
				//count_all++;
			}
		}
	}
	std::cout<<"voxel_size:"<<voxel_size<<"\n";
	std::cout<<"voxel_size:"<<tm_cls.get_time_now()<<"\n";
	//convert voxel[][][] to pointcloud and write index
}
void detect_objects::publish_voxel_pcl(int& voxel_size){
	selfvoxel_cloud->points.clear();
	selfvoxel_cloud->height=1;
	selfvoxel_cloud->width=voxel_size;
	selfvoxel_cloud->points.resize(voxel_size);
	int vn=0;
	std::cout<<"reserved\n";
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				if(voxel_point[nz][nx][ny].s!=0){
					//index_points[nz][nx][ny]=(int)selfvoxel_cloud->points.size();
					selfvoxel_cloud->points[vn].x=voxel_point[nz][nx][ny].x;
					selfvoxel_cloud->points[vn].y=voxel_point[nz][nx][ny].y;
					selfvoxel_cloud->points[vn++].z=voxel_point[nz][nx][ny].z;
				}
			}
		}
	}

	sensor_msgs::PointCloud2 edit_cloud;
	pcl::toROSMsg (*selfvoxel_cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
	pc_pub8.publish(edit_cloud);
	std::cout<<"finish_func\n";
	std::cout<<"finish_func:"<<tm_cls.get_time_now()<<"\n";
}
void detect_objects::clusterig_selfvoxel(void){
	//clustering 

	std::vector<pcl::PointXYZ> cluster_elements;
	//std::vector<Point3f1i> cluster_elements;
	std::vector<cv::Point3i> cluster_elements_num;
	pcl::PointXYZ cluster_elements_temp;
	cv::Point3i cen_temp;

	cur_cluster.reserve(voxel_size);
	cluster_elements.reserve(voxel_size);
	cluster_elements_num.reserve(voxel_size);

	
	//init cur_clusted_index
	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){
				pre_clusted_index[nz][nx][ny]=cur_clusted_index[nz][nx][ny];
				cur_clusted_index[nz][nx][ny]=-1;
			}
		}
	}
	pre_cluster=cur_cluster;
	cur_cluster.clear();

	//clustering process
	float clustering_distance=0.04*1.5;
	int serch_range_x=(int)(clustering_distance/voxel_size_x)+1;
	int serch_range_y=(int)(clustering_distance/voxel_size_y)+1;
	int serch_range_z=(int)(clustering_distance/voxel_size_z)+1;
	float edis;
	int cluster_size=0;
	std::cout<<"clustering process\n";


	for(int nz=0;nz<map_size_nz;nz++){
		for(int nx=0;nx<map_size_nx;nx++){
			for(int ny=0;ny<map_size_ny;ny++){

//	for(int tsk_n=0;tsk_n<clst_tsk.size();tsk_n++){
		//nz=clst_tsk[tsk_n].nz;
		//nx=clst_tsk[tsk_n].nx;
		//ny=clst_tsk[tsk_n].ny;
				if(voxel_point[nz][nx][ny].s==0||cur_clusted_index[nz][nx][ny]!=-1){//already clusted or point nothing
					//std::cout<<"("<<nz<<","<<nx<<","<<ny<<")\n";
					continue;//skip
				}

				/*
				std::cout<<voxel_point[nz][nx][ny].x<<","
									<<voxel_point[nz][nx][ny].y<<","
									<<voxel_point[nz][nx][ny].z<<"\n";
				*/
				
				cluster_elements_temp.x=voxel_point[nz][nx][ny].x;
				cluster_elements_temp.y=voxel_point[nz][nx][ny].y;
				cluster_elements_temp.z=voxel_point[nz][nx][ny].z;
				cluster_elements.push_back(cluster_elements_temp);
				
				//cluster_elements.push_back(voxel_point[nz][nx][ny]);
				cen_temp.z=nz;
				cen_temp.x=nx;
				cen_temp.y=ny;
				cluster_elements_num.push_back(cen_temp);
				cur_clusted_index[nz][nx][ny]=(int)cur_cluster.size();
				//std::cout<<"nz,nx,ny):("<<nz<<","<<nx<<","<<ny<<")\n";
				for(int k=0;k<cluster_elements.size();k++){
				  int cen_nz,cen_nx,cen_ny;
					//convert_xzy_nxzy(cluster_elements_num[k].z,cluster_elements_num[k].x,cluster_elements_num[k].y,cen_nz,cen_nx,cen_ny);
					cen_nz=cluster_elements_num[k].z;
					cen_nx=cluster_elements_num[k].x;
					cen_ny=cluster_elements_num[k].y;
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
								if(sry<0||sry>=map_size_ny||voxel_point[srz][srx][sry].s==0
									||cur_clusted_index[srz][srx][sry]!=-1){//outrange of array y or point nothing
									continue;
								}
								//Euclid distance
								//edis=culclate_euclid_distance(cluster_elements[k],voxel_point[srz][srx][sry]);
								//Chebyshev distance
								edis=culclate_chebyshev_distance(cluster_elements[k],voxel_point[srz][srx][sry]);
								
								if(edis<=clustering_distance){//clust
									//cluster_elements.push_back(voxel_point[srz][srx][sry]);
									cluster_elements_temp.x=voxel_point[srz][srx][sry].x;
									cluster_elements_temp.y=voxel_point[srz][srx][sry].y;
									cluster_elements_temp.z=voxel_point[srz][srx][sry].z;
									cluster_elements.push_back(cluster_elements_temp);

									cen_temp.z=srz;
									cen_temp.x=srx;
									cen_temp.y=sry;
									cluster_elements_num.push_back(cen_temp);
									cur_clusted_index[srz][srx][sry]=(int)cur_cluster.size();
									cluster_size++;
								}
							}//end for search range y
						}//end for search range x
					}//end for search range z
				}//end for k
				cur_cluster.push_back(cluster_elements);
				cluster_elements.clear();
				cluster_elements_num.clear();
	//}
	
			}//end for ny
		}//end for nx
	}//end for nz
	
	std::cout<<"cluster_size:"<<cluster_size<<"\n";
		std::cout<<"cluster_size:"<<tm_cls.get_time_now()<<"\n";
	//draw clusters
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB temp_point;
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト	

	//reserve clusted_cloud
	//clusted_cloud->points.resize(voxel_size);//cluster_size);
	clusted_cloud->points.reserve(voxel_size);//cluster_size);
	clusted_cloud->height=1;
	clusted_cloud->width=voxel_size;
	int ccn=0;	
	pcl::PointXYZRGB cloud_temp;

	float volume_threshold=0.1*0.1*0.1;
	float one_point_volume=voxel_size_x*voxel_size_z*voxel_size_y;
	for(int cn=0;cn<cur_cluster.size();cn++){
		if(one_point_volume*(int)cur_cluster[cn].size()>volume_threshold){
			for(int cen=0;cen<cur_cluster[cn].size();cen++){
				/*
				clusted_cloud->points[ccn].x=cluster[cn][cen].x;
				clusted_cloud->points[ccn].y=cluster[cn][cen].y;
				clusted_cloud->points[ccn].z=cluster[cn][cen].z;
				clusted_cloud->points[ccn].r=colors[j%12][0];
				clusted_cloud->points[ccn].g=colors[j%12][1];
				clusted_cloud->points[ccn].b=colors[j%12][2];
				
				ccn++;
				*/
				cloud_temp.x=cur_cluster[cn][cen].x;
				cloud_temp.y=cur_cluster[cn][cen].y;
				cloud_temp.z=cur_cluster[cn][cen].z;
				cloud_temp.r=colors[j%12][0];
				cloud_temp.g=colors[j%12][1];
				cloud_temp.b=colors[j%12][2];
				clusted_cloud->points.push_back(cloud_temp);
				

			}
			j++;
		}
		else{
			for(int cen=0;cen<cur_cluster[cn].size();cen++){
				/*
				clusted_cloud->points[ccn].x=cur_cluster[cn][cen].x;
				clusted_cloud->points[ccn].y=cur_cluster[cn][cen].y;
				clusted_cloud->points[ccn].z=cur_cluster[cn][cen].z;
				clusted_cloud->points[ccn].r=0;
				clusted_cloud->points[ccn].g=0;
				clusted_cloud->points[ccn].b=0;
				ccn++;
				
				*/
				cloud_temp.x=cur_cluster[cn][cen].x;
				cloud_temp.y=cur_cluster[cn][cen].y;
				cloud_temp.z=cur_cluster[cn][cen].z;
				cloud_temp.r=0;
				cloud_temp.g=0;
				cloud_temp.b=0;
				clusted_cloud->points.push_back(cloud_temp);
						
			}		
		}
	}
	//clusted_cloud->points.resize(clusted_cloud->points.size());
	clusted_cloud->width=clusted_cloud->points.size();
	std::cout<<"clusted_cloud->points:"<<clusted_cloud->points.size()<<"\n";
	std::cout<<"clusted_cloud->width:"<<clusted_cloud->width<<"\n";




	//publish point cloud
	sensor_msgs::PointCloud2 edit_cloud2;
  pcl::toROSMsg (*clusted_cloud, edit_cloud2);
	edit_cloud2.header.frame_id="/zed_current_frame";
  pc_pub9.publish(edit_cloud2);

	//cluster.clear();
}





