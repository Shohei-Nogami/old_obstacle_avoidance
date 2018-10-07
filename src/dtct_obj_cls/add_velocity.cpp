#include"detect_objects_class.h"

void detect_objects::subscribe_opticalflow(void){
	queue_optflw.callOne(ros::WallDuration(1));
}
void detect_objects::opticalflow_callback(const obst_avoid::vel3d::ConstPtr& msg){
	vX.pt=msg->pt;
	vX.vel=msg->vel;
}
bool detect_objects::add_velocity_to_cluster(void){
	if(!vX.pt.size())
	{
		return false;
	}
	std::cout<<"vX.pt,vel:"<<vX.pt.size()<<","<<vX.vel.size()<<"\n";
	//std::vector< std::vector<pcl::PointXYZ> > cluster_vel_elm;
	pcl::PointXYZ vel_element;
	//cluster_vel_elm.resize(cur_cluster.size());
	cluster_vel_elm.reserve(cur_cluster.size());
	int h,w;
	int nx,nz,ny;
	int cn;
	std::cout<<"vX.pt[0],vel[0]:"<<vX.pt[0]<<","<<vX.vel[0]<<"\n";
	for(int k=0;k<vX.pt.size();k++){
		//std::cout<<"for \n";
		h=vX.pt[k].h;
		w=vX.pt[k].w;
		//std::cout<<",h,w:"<<h<<","<<w<<"\n";
		nx=cur_index_vxl[h][w].nx;
		ny=cur_index_vxl[h][w].ny;
		nz=cur_index_vxl[h][w].nz;
		//std::cout<<"nx,ny,nz,h,w,cn:"<<nx<<","<<ny<<","<<nz<<","<<h<<","<<w<<",";
	
		if(nz!=-1)
		{
			//std::cout<<"nx,ny,nz:"<<nx<<","<<ny<<","<<nz<<"\n";
			cn=cur_clusted_index[nz][nx][ny];
			//std::cout<<cn<<"\n";
			//std::cout<<"h,w,,cn:"<<h<<","<<w<<","<<cn<<"\n";
			if(cn!=-1)
			{
				vel_element.x=vX.vel[k].x;
				vel_element.y=vX.vel[k].y;
				vel_element.z=vX.vel[k].z;

				cluster_vel_elm[cn].push_back(vel_element);
			}
		}
		//std::cout<<":for end\n";
		//std::cout<<"k,vXsize:"<<k<<","<<vX.pt.size()<<"\n";
	}
	//std::cout<<"cur_cluster.size():"<<cur_cluster.size()<<"\n";
	/*
	for(int i=0;i<cur_cluster.size();i++){
		std::cout<<"cluster_vel_elm["<<i<<"]:"<<cluster_vel_elm[i].size()<<"\n";
		
	}
	*/
	std::cout<<"end\n";
	return true;
}

void detect_objects::estimate_velocity_of_cluster(void)
{
	//std::vector<pcl::PointXYZ> cluster_vel;
	cluster_vel.resize(cur_cluster.size());
	std::vector<pcl::PointXYZ> cluster_vel_dsp;
	cluster_vel_dsp.resize(cur_cluster.size());
	for(int i=0;i<cur_cluster.size();i++)
	{
		int n=0;
		cluster_vel[i].x=0;
		cluster_vel[i].y=0;
		cluster_vel[i].z=0;
		cluster_vel_dsp[i].x=0;
		cluster_vel_dsp[i].y=0;
		cluster_vel_dsp[i].z=0;
		for(int k=0;k<cluster_vel_elm[i].size();k++)
		{
			if(std::sqrt(std::pow(cluster_vel_elm[i][k].x,2.0)+std::pow(cluster_vel_elm[i][k].z,2.0))<1.1)//object speed <4.0km/h
			{
				cluster_vel[i].x+=cluster_vel_elm[i][k].x;
				//cluster_vel.y+=cluster_vel_elm.y;
				cluster_vel[i].z+=cluster_vel_elm[i][k].z;
				n++;
			}
		}
		if(n<=0)
		{
			//std::cout<<"if(n<=0)\n";
			continue;
		}
		cluster_vel[i].x=cluster_vel[i].x/n;
//		cluster_vel.y=cluster_vel.y/n;
		cluster_vel[i].z=cluster_vel[i].z/n;
		
		for(int k=0;k<cluster_vel_elm[i].size();k++)
		{
			if(std::sqrt(std::pow(cluster_vel_elm[i][k].x,2.0)+std::pow(cluster_vel_elm[i][k].z,2.0))<1.1)//object speed <4.0km/h
			{
				cluster_vel_dsp[i].x+=std::pow( cluster_vel[i].x-cluster_vel_elm[i][k].x , 2.0);
				//cluster_vel_dsp[i].y+=std::pow( cluster_vel[i].y-cluster_vel_elm[i][k].y , 2.0);
				cluster_vel_dsp[i].z+=std::pow( cluster_vel[i].z-cluster_vel_elm[i][k].z , 2.0);
			}
		}
		cluster_vel_dsp[i].x=std::sqrt(cluster_vel_dsp[i].x/n);
//		cluster_vel_dsp.y=std::sqrt(cluster_vel_dsp.y/n);
		cluster_vel_dsp[i].z=std::sqrt(cluster_vel_dsp[i].z/n);
		if(cluster_vel_dsp[i].x<cluster_vel[i].x*0.1&&cluster_vel_dsp[i].z<cluster_vel[i].z*0.1)
		{
			std::cout<<"cluster["<<i<<"]:("<<cluster_vel[i].x<<","<<cluster_vel[i].z<<")\n";
			
		}
		else{
		cluster_vel[i].x=0;
		cluster_vel[i].y=0;
		cluster_vel[i].z=0;
		}
	}
	
	/*
	for(int i=0;i<cluster.size();i++)
	{
		if(!std::isnan(cluster_vel[i].x))
		{
			std::cout<<"cluster(num,speed(x,z)):("<<i<<",speed("<<cluster_vel[i].x<<","<<cluster_vel[i].z<<"))\n";//when speed is nan, cluster_vel[i].size() is 0
		}
	}
	*/
}

void detect_objects::subsuctibe_matching(void){
	queue_matching.callOne(ros::WallDuration(1));
}

void detect_objects::matching_callback(const obst_avoid::matching::ConstPtr& msg)
{
	match_msg.pre=msg->pre;
	match_msg.cur=msg->cur;	
}
void detect_objects::matching_cluster(void)
{
	int ccn,pcn;
	int h,w;
	int nx,ny,nz;
	std::vector< std::vector<int> > cmatch_temp;
	//std::vector<int> cmatch;
	cmatch.clear();
	cmatch.resize(pre_cluster.size());
	cmatch_temp.resize(pre_cluster.size());
	for(int k=0;k<pre_cluster.size();k++)
	{
		cmatch_temp[k].reserve(match_msg.pre.size());
	}	
	for(int k=0;k<match_msg.pre.size();k++)
	{
		//set pre point
		h=match_msg.pre[k].y;
		w=match_msg.pre[k].x;
		//std::cout<<",h,w:"<<h<<","<<w<<"\n";
		//convert point to num of voxelgrid
		nx=pre_index_vxl[h][w].nx;
		ny=pre_index_vxl[h][w].ny;
		nz=pre_index_vxl[h][w].nz;
		//if voxel grid exists
		if(nz!=-1)
		{
			//set pre cluster num
			pcn=pre_clusted_index[nz][nx][ny];
			//if cluster num exists
			if(pcn!=-1)
			{	
				//set cur points
				h=match_msg.cur[k].y;
				w=match_msg.cur[k].x;
				///convert point to num of voxelgrid
				nx=cur_index_vxl[h][w].nx;
				ny=cur_index_vxl[h][w].ny;
				nz=cur_index_vxl[h][w].nz;
				//if voxel grid exists
				if(nz!=-1)
				{
					//set cur cluster num
					ccn=cur_clusted_index[nz][nx][ny];
					//if cluster num exists
					if(ccn!=-1)
					{
						//add index to cur cluster num
						cmatch_temp[pcn].push_back(ccn);
					}
				}
			}
		}
		//std::cout<<":for end\n";
		//std::cout<<"k,vXsize:"<<k<<","<<vX.pt.size()<<"\n";
	}
	for(int i=0;i<pre_cluster.size();i++)
	{
		//std::cout<<"for\n";
		std::vector<int> index_num;
		index_num.resize(cur_cluster.size());
		for(int k=0;k<cur_cluster.size();k++)
		{
			index_num[ k ]=0;
		}
		for(int k=0;k<cmatch_temp[i].size();k++)
		{
			index_num[ cmatch_temp[i][k] ]++;
		}
		//std::cout<<"a\n";
		int max=0;
		//int max_n=0;
		cmatch[i]=0;
		for(int n=0;n<cur_cluster.size();n++)
		{
			if(max<index_num[n])
			{	
				max=index_num[n];
				//max_n=n;
				cmatch[i]=n;
			}
		}
		//std::cout<<"("<<pre_cluster.size()<<")"<<"cmatch["<<i<<"]:"<<cmatch[i]<<"\n";
	}	
}

bool detect_objects::estimate_velocity_of_cluster_by_gp(double& dt)
{
	//prev_cluster,cur_cluster
	//culculate gravity points of current cluster
	
	//std::vector<pcl::PointXYZ> cur_gp;
	
	pre_gp=cur_gp;
	cur_gp.resize(cur_cluster.size());
	std::cout<<"a\n";
	for(int i=0;i<cur_cluster.size();i++)
	{
		cur_gp[i].x=0;
		cur_gp[i].y=0;
		cur_gp[i].z=0;
		for(int k=0;k<cur_cluster[i].size();k++)
		{
			cur_gp[i].x+=cur_cluster[i][k].x;
			cur_gp[i].y+=cur_cluster[i][k].y;
			cur_gp[i].z+=cur_cluster[i][k].z;
			
		}
		cur_gp[i].x=cur_gp[i].x/(int)cur_cluster[i].size();
		cur_gp[i].y=cur_gp[i].y/(int)cur_cluster[i].size();
		cur_gp[i].z=cur_gp[i].z/(int)cur_cluster[i].size();
		
	}
	std::cout<<"aa\n";
	if(!pre_cluster.size())
	{
		return false;
	}
	//holding on
	//std::vector<pcl::PointXYZ> cluster_vel_gp;
	cluster_vel_gp.clear();
	cluster_vel_gp.resize(cur_cluster.size());
	for(int i=0;i<pre_cluster.size();i++)
	{
		std::cout<<"cluster_vel_gp.size():"<<cluster_vel_gp.size()<<"\n";
		std::cout<<"cmatch["<<i<<"]:"<<cmatch[i]<<"\n";
		std::cout<<"cluster_vel_gp[cmatch[i]].x:"<<cluster_vel_gp[cmatch[i]].x<<"\n";
		std::cout<<"cur_gp[cmatch[i]]:"<<cur_gp[cmatch[i]]<<"\n";
	  cluster_vel_gp[cmatch[i]].x=(cur_gp[cmatch[i]].x-pre_gp[i].x)/dt;
	  cluster_vel_gp[cmatch[i]].y=(cur_gp[cmatch[i]].y-pre_gp[i].y)/dt;
	  cluster_vel_gp[cmatch[i]].z=(cur_gp[cmatch[i]].z-pre_gp[i].z)/dt;
	}
	
	std::cout<<"aaa\n";
	for(int i=0;i<cur_cluster.size();i++)
	{
	  if(std::isnan(cluster_vel_gp[i].x))
	  {
	    cluster_vel_gp[i].x=0;
	    cluster_vel_gp[i].y=0;
	    cluster_vel_gp[i].z=0;
	  }
	}
	
	
	
	return true;
}

void detect_objects::draw_velocity(cv::Mat& image)
{
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	view_vel_image=image.clone();
	for(int i=0;i<cur_cluster.size();i++)
	{
		float volume_threshold=0.1*0.1*0.1;
		float one_point_volume=voxel_size_x*voxel_size_z*voxel_size_y;
		if(one_point_volume*(int)cur_cluster[i].size()>volume_threshold
				//&&!std::isnan(cluster_vel[i].x)
				&&!std::isnan(cluster_vel_gp[i].x)
				//std::sqrt(std::pow(cluster_vel[i].x,2.0)+std::pow(cluster_vel[i].z,2.0))>0.1
				//std::sqrt(std::pow(cluster_vel_gp[i].x,2.0)+std::pow(cluster_vel_gp[i].z,2.0))>0.1
				&&std::sqrt(std::pow(cluster_vel_gp[i].x,2.0)+std::pow(cluster_vel_gp[i].z,2.0))<1.1
				&&std::sqrt(std::pow(cluster_vel_gp[i].x,2.0)+std::pow(cluster_vel_gp[i].z,2.0))>0.05
				)
		{
			std::cout<<"cluster_vel_gp[i]:"<<cluster_vel_gp[i]<<"\n";
			std::string vel_string_x,vel_string_z,vel_string;
			//vel_string_x=std::to_string(cluster_vel[i].x);//(int)(cluster_vel[i].x*1000)/(float)1000);
			//vel_string_z=std::to_string(cluster_vel[i].z);//(int)(cluster_vel[i].z*1000)/(float)1000);
			vel_string_x=std::to_string(cluster_vel_gp[i].x);//(int)(cluster_vel[i].x*1000)/(float)1000);
			vel_string_z=std::to_string(cluster_vel_gp[i].z);//(int)(cluster_vel[i].z*1000)/(float)1000);
			vel_string="("+vel_string_x.substr(0,5)+","+vel_string_z.substr(0,5)+")";
			cv::Point2i gp;
			gp.x=0;
			gp.y=0;
			for(int k=0;k<cur_cluster[i].size();k++)
			{
				gp.x+=-cur_cluster[i][k].y/cur_cluster[i][k].x*f;
				gp.y+=(cur_cluster[i][k].z-0.23/*0.4125*/)/cur_cluster[i][k].x*f;

				//view_vel_image.at<cv::Vec3b>(gp.y,gp.x)
						
			}
			gp.x=(int)(gp.x/(int)cur_cluster[i].size())+width/2-100;
			gp.y=height/2-(int)(gp.y/(int)cur_cluster[i].size());
			//std::cout<<"gp:"<<gp<<"\n";
			
			cv::putText(view_vel_image,vel_string,gp,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,0), 2, CV_AA);
			//cv::putText(view_vel_image,"HidakaLab",cv::Point(30,30),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 2, CV_AA);
			//j++;
		}
	}
	//publish vel image
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=view_vel_image.clone();
	pub1.publish(publish_cvimage->toImageMsg());
}

void detect_objects::clear_velocity(void)
{
	vX.pt.clear();
	vX.vel.clear();
	cluster_vel.clear();
}



