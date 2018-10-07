#include"detect_objects_class.h"



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


