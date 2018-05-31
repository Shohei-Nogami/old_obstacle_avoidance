#include"estimate_velocity_class.h"

estimate_velocity::estimate_velocity()
	:it_pub1(nh_pub1)
{	
	nh_cluster.setCallbackQueue(&queue_cluster);
	sub_cluster=nh_cluster.subscribe("/cluster",1,&estimate_velocity::cluster_callback,this);
	//nh_optflw.setCallbackQueue(&queue_optflw);
	//sub_optflw=nh_optflw.subscribe("objects_velocity",1,&estimate_velocity::opticalflow_callback,this);
	
	nh_match.setCallbackQueue(&queue_match);
	sub_match=nh_match.subscribe("cluster_matching_index",1,&estimate_velocity::match_index_callback,this);

  pc_pub = nh_pubpcl.advertise<sensor_msgs::PointCloud2>("clusted_cloud", 1);
	pub1=it_pub1.advertise("vel_image",1);//test string

	pub2 = nh_pubpcl.advertise<obst_avoid::cluster_with_vel>("cluster_with_vel", 1);
	

	//--reserve memory
	//index of maching
	match_msg.pre.reserve(width*height);
	match_msg.cur.reserve(width*height);
	cluster_match.reserve(width*height);

	for(int h=0;h<height;h++)
	{
		for(int w=0;w<width;w++)
		{
			cur_cluster_index[h][w]=-1;
			pre_cluster_index[h][w]=-1;
		}
	}
	//std::cout<<"cur_cluster_index.size():"<<cur_cluster_index.size()<<"\n";
	//std::cout<<"cur_cluster_index[0].size():"<<cur_cluster_index[0].size()<<"\n";
}
estimate_velocity::~estimate_velocity()
{


}
void estimate_velocity::subsuctibe_cluster(void)
{
	queue_cluster.callOne(ros::WallDuration(1));
}
void estimate_velocity::cluster_callback(const obst_avoid::cluster::ConstPtr& msg)
{
	cur_cluster.dX=msg->dX;
	cur_cluster.t=msg->t;
	cur_cluster.clst=msg->clst;
}
void estimate_velocity::subsuctibe_match_index(void){
	queue_match.callOne(ros::WallDuration(1));
}

void estimate_velocity::match_index_callback(const obst_avoid::matching::ConstPtr& msg)
{
	//match_msg.pre.resize(msg->pre.size());
	//match_msg.cur.resize(msg->cur.size());

	//std::copy(match_msg.pre.begin(),match_msg.pre.end(),msg->pre.begin());
	//std::copy(match_msg.cur.begin(),match_msg.cur.end(),msg->cur.begin());
	//std::copy(match_msg.pre.begin(),match_msg.pre.end(),std::back_inserter(msg->pre));
	//std::copy(match_msg.cur.begin(),match_msg.cur.end(),std::back_inserter(msg->cur));

	match_msg.pre=msg->pre;
	match_msg.cur=msg->cur;	
}
bool estimate_velocity::matching_cluster(void)
{
	//std::cout<<"aa\n";
	if(!match_msg.pre.size()||!cur_cluster.clst.size())
	{
		return false;
	}
	pre_cluster_match.resize(cluster_match.size());
	for(int i=0;i<cluster_match.size();i++)
	{
		pre_cluster_match[i]=cluster_match[i];
	}
	cluster_match.clear();
	cluster_match.resize(cur_cluster.clst.size());

	pre_cluster=cur_cluster;
	
	if(track_n.size())
	{
		pre_track_n.resize(track_n.size());
		for(int i=0;i<pre_track_n.size();i++)
		{
		  pre_track_n[i]=track_n[i];
		}
	}
	track_n.resize(cur_cluster.clst.size());

	//std::cout<<"aaaaaa\n";
	//cluster number in cluster_index
	//init index 
	for(int h=0;h<height;h++)
	{
		for(int w=0;w<width;w++)
		{
			cur_cluster_index[h][w]=-1;
		}
	}
	//std::cout<<"aabb\n";
	//write index
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
			int h=cur_cluster.clst[i].pt[k].y*ksize;
			int w=cur_cluster.clst[i].pt[k].x*ksize;
			//int h=cur_cluster.clst[i].pt[k].y*ksize+ksize/2;
			//int w=cur_cluster.clst[i].pt[k].x*ksize+ksize/2;
			for(int v=h;v<h+ksize;v++)
			{
				for(int u=w;u<w+ksize;u++)
				{
					cur_cluster_index[v][u]=i;
					//std::cout<<"i:"<<i<<"\n";
				}
			}
			
		}
	}
	
	if(!pre_cluster.clst.size())
	{
		return false;
	}
	std::cout<<"aabbbb\n";
	//matching exist bagggg
	//
	//std::vector<int> match_n;
	
	//int* match_n(NULL);

	//match_n = new int[(int)pre_cluster.clst.size()];
//	int *match_n;
//	match_n=new int[(int)pre_cluster.clst.size()];
	//int match_n[(int)pre_cluster.clst.size()];
//	match_n.clear();
//	match_n.reserve(pre_cluster.clst.size());

//マッチングポイント探索方法
	match_n.resize((int)cur_cluster.clst.size());
	
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		match_n[i].resize((int)pre_cluster.clst.size());
		for(int k=0;k<pre_cluster.clst.size();k++)
		{
			match_n[i][k]=0;
		}
	}
	//std::cout<<"match init\n";
	for(int k=0;k<match_msg.cur.size();k++)
	{
		
		if(match_msg.cur[k].x==-1)
		{
		  continue;
		}
		int ch=k/width;
		int cw=k%width;
		if(cur_cluster_index[ch][cw]==-1)
		{
		  continue;
		}
		int ph=match_msg.cur[k].y;  
		int pw=match_msg.cur[k].x;
		
		if(pre_cluster_index[ph][pw]==-1)
		{
		  continue;
		}
		int cur_index=cur_cluster_index[ch][cw];
		int pre_index=pre_cluster_index[ph][pw];
		//std::cout<<"match_n["<<cur_index<<"]["<<pre_index<<"]\n";
		match_n[cur_index][pre_index]++;
		
	}


	matched.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		matched[i]=false;
		cluster_match[i]=-1;
	}

	for(int k=0;k<pre_cluster.clst.size();k++)
	{
		//select best match
		int max_n=0;
		int max_i=-1;
		for(int i=0;i<cur_cluster.clst.size();i++)
		{
			
			if(max_n<match_n[i][k]&&!matched[i]
				&&( (float)( std::abs( (int)pre_cluster.clst[i].pt.size() - (int)cur_cluster.clst[i].pt.size() ))/(int)cur_cluster.clst[i].pt.size()<2 )
			)
			{
				max_n=match_n[i][k];
				max_i=i;
			}
		}
		if(max_i!=-1)
		{
			cluster_match[max_i]=k;//cur -> pre 
			matched[max_i]=true;
		}
		//std::cout<<"1111\n";
	}
	//std::cout<<"count:"<<count<<"\n";
	//std::cout<<"end\n";
	//delete match_n;
	/*
	for(int i=0;i<4;i++)
	{
		for(int k=0;k<4;k++)
		{
			std::cout<<"cur_cluster_index:("<<i<<","<<k<<"):("<<cur_cluster_index[i][k]<<"\n";			
		}
	}
	*/
	//pre_cluster_index.resize(cur_cluster_index.size());
	//std::copy(pre_cluster_index.begin(),pre_cluster_index.end(),cur_cluster_index.begin());
	//for(int h=0;h<cur_cluster_index.size();h++)
	for(int h=0;h<height;h++)
	{
		//for(int w=0;w<cur_cluster_index[h].size();w++)
		for(int w=0;w<width;w++)
		{
			pre_cluster_index[h][w]=cur_cluster_index[h][w];
		}
	}
	/*
	for(int i=0;i<4;i++)
	{
		for(int k=0;k<4;k++)
		{
			std::cout<<"pre_cluster_index:("<<i<<","<<k<<"):("<<pre_cluster_index[i][k]<<"\n";			
		}
	}
	*/
	//delete match_n;
	//std::vector<int>().swap(match_n);
	return true;
}

bool estimate_velocity::estimate_velocity_of_cluster(void)
{
	cur_cluster_size.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		 cur_cluster_size[i]=0;
	}
	if(cur_gp.size())
	{
    pre_gp=cur_gp;
	}	
	cur_gp.clear();
	cur_gp.resize(cur_cluster.clst.size());

	pre_vel.resize(vel.size());
	for(int i=0;i<vel.size();i++)
	{
		pre_vel[i]=vel[i];
	}
	vel.clear();
	vel.resize(cur_cluster.clst.size());
	for(int i=0;i<vel.size();i++)
	{
		vel[i].x=0;
		vel[i].y=0;
		vel[i].z=0;
	}
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		cur_gp[i].x=0;
		cur_gp[i].y=0;
		cur_gp[i].z=0;

		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
			//cur_gp[i].x+=(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
			////cur_gp[i].y+=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].z)/f+0.4125;
			//cur_gp[i].z+=cur_cluster.clst[i].pt[k].z;
			cur_gp[i].x+=(cur_cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cur_cluster.clst[i].pt[k].z/f;
			//cur_gp[i].y+=((height/2-cur_cluster.clst[i].pt[k].y*ksize+ksize/2)*cur_cluster.clst[i].z)/f+0.4125;
			cur_gp[i].z+=cur_cluster.clst[i].pt[k].z;
			
			//add cluster size
			cur_cluster_size[i]+=std::pow(ksize*cur_cluster.clst[i].pt[k].z/f,2.0);
		}
		cur_gp[i].x=cur_gp[i].x/(int)cur_cluster.clst[i].pt.size();
		//cur_gp[i].y=cur_gp[i].y/(int)cur_cluster.clst[i].pt.size();
		cur_gp[i].z=cur_gp[i].z/(int)cur_cluster.clst[i].pt.size();

		
	}
	if(!pre_gp.size())
	{
		return false;
	}
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		int match_num = cluster_match[i];
		track_n[i]=0;
		//std::cout<<"cluster_match["<<i<<"]:"<<cluster_match[i]<<"\n";
		if(match_num==-1)
		{
			continue;
		}
		vel[i].x = (cur_gp[i].x-pre_gp[match_num].x)/cur_cluster.t-cur_cluster.dX.x/cur_cluster.t;
		vel[i].y = 0;//(cur_gp[i].y-pre_gp[i].y)/cur_cluster.t-cur_cluster.dX.y/cur_cluster.t;
		vel[i].z = (cur_gp[i].z-pre_gp[match_num].z)/cur_cluster.t-cur_cluster.dX.z/cur_cluster.t;
		int k=0;
		float T=0.1;
		if(track_n[i]>0)
		{
			//std::cout<<"i,k:"<<i<<","<<k<<"\n";
			vel[i].x=(vel[i].x*cur_cluster.t+pre_vel[match_num].x*T)/(T+cur_cluster.t);
			vel[i].y=(vel[i].y*cur_cluster.t+pre_vel[match_num].y*T)/(T+cur_cluster.t);
			vel[i].z=(vel[i].z*cur_cluster.t+pre_vel[match_num].z*T)/(T+cur_cluster.t);
		}
		track_n[i]=pre_track_n[k]+1;

		
		
		//std::cout<<"cur_gp["<<i<<"]:"<<cur_gp[i]<<"\n";
		//std::cout<<"pre_gp["<<i<<"]:"<<pre_gp[i]<<"\n";
		//std::cout<<"cur_cluster.t:"<<cur_cluster.t<<"\n";
		std::cout<<"vel["<<i<<"]("<< track_n[i] << "):"<<vel[i]<<"\n";
		//std::cout<<"cur_cluster_size["<<i<<"]:"<<cur_cluster_size[i]<<"\n";
	}
}

void estimate_velocity::predict_cluster(void)
{

}

void estimate_velocity::draw_velocity(cv::Mat& image)
{

	view_vel_image=image.clone();

	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	if(image.empty())
	{
		std::cout<<"image.empty()\n";
		return ;
	}
	std::cout<<"aa\n";
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		if(!std::isnan(vel[i].x)&&vel[i].x!=0
				&&std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>0.1
				&&std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))<1.1
				&&&track_n[i]>0
				&&vel[i].z<0
				)
		{
			std::string vel_string_x,vel_string_z,vel_string;
			vel_string_x=std::to_string(vel[i].x);//(int)(cluster_vel[i].x*1000)/(float)1000);
			vel_string_z=std::to_string(vel[i].z);//(int)(cluster_vel[i].z*1000)/(float)1000);
			vel_string="("+vel_string_x.substr(0,5)+","+vel_string_z.substr(0,5)+")";
			cv::Point2i gp;
			gp.x=0;
			gp.y=0;
			for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
			{
				//gp.x+=cur_cluster.clst[i].pt[k].x*ksize;
				//gp.y+=cur_cluster.clst[i].pt[k].y*ksize;
				gp.x+=cur_cluster.clst[i].pt[k].x*ksize+ksize/2;
				gp.y+=cur_cluster.clst[i].pt[k].y*ksize+ksize/2;

				//view_vel_image.at<cv::Vec3b>(gp.y,gp.x)
						
			}
			gp.x=(int)(gp.x/(int)cur_cluster.clst[i].pt.size());
			gp.y=(int)(gp.y/(int)cur_cluster.clst[i].pt.size());
			//std::cout<<"gp:"<<gp<<"\n";
			
			cv::putText(view_vel_image,vel_string,gp,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,0), 2, CV_AA);
			//cv::putText(view_vel_image,"HidakaLab",cv::Point(30,30),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 2, CV_AA);
			//j++;
		}
	}
	//std::cout<<"aa\n";
	//publish vel image
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=view_vel_image.clone();
	pub1.publish(publish_cvimage->toImageMsg());
}

void estimate_velocity::publish_pointcloud(void)
{
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	clusted_cloud->points.clear();
	clusted_cloud->points.reserve(width*height);

	pcl::PointXYZRGB cloud_temp;

	for(int i=0;i<cur_cluster.clst.size();i++)
	{

		
		for(int t=0;t<4;t++)
		{
			if(std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>1.1
			//|| 
			)
			{
				continue;
			}
			for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
			{
				//cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
				//cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
				cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cur_cluster.clst[i].pt[k].z/f;
				cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize+ksize/2)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
				cloud_temp.x=cur_cluster.clst[i].pt[k].z;
				cloud_temp.r=colors[i%12][0];
				cloud_temp.g=colors[i%12][1];
				cloud_temp.b=colors[i%12][2];
				cloud_temp.x+=vel[i].z*t;
		    cloud_temp.y+=-vel[i].x*t;			
		    cloud_temp.z+=vel[i].y*t;
		
				clusted_cloud->points.push_back(cloud_temp);
			}
		}

		if(std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))<0.1){
			for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		  {
		      //cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
		      //cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
		      cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cur_cluster.clst[i].pt[k].z/f;
		      cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize+ksize/2)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
		      cloud_temp.x=cur_cluster.clst[i].pt[k].z;
		      cloud_temp.r=colors[i%12][0];
		      cloud_temp.g=colors[i%12][1];
		      cloud_temp.b=colors[i%12][2];

		      //cloud_temp.x+=vel[i].z*3;
		      //cloud_temp.y+=-vel[i].x*3;			
		     	//cloud_temp.z+=vel[i].y*3;

		      cloud_temp.z-=1.5;

		      clusted_cloud->points.push_back(cloud_temp);
		  }
		}

		for(int t=0;t<4;t++)
		{
			if(track_n[i]<1||std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>1.1
				||cur_cluster_size[i]>1.0*1.0||cur_cluster_size[i]<0.1*0.1
			)
			{
				continue;
			}
			for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
			{
				//cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
				//cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
				cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cur_cluster.clst[i].pt[k].z/f;
				cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize+ksize/2)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
				cloud_temp.x=cur_cluster.clst[i].pt[k].z;
				cloud_temp.r=colors[i%12][0];
				cloud_temp.g=colors[i%12][1];
				cloud_temp.b=colors[i%12][2];
				cloud_temp.x+=vel[i].z*t;
		    cloud_temp.y+=-vel[i].x*t;			
		    cloud_temp.z+=vel[i].y*t;
				
		    cloud_temp.y+=5;		

				clusted_cloud->points.push_back(cloud_temp);
			}		
		
		}
    int match_num = cluster_match[i];
    //std::cout<<"cluster_match["<<i<<"]:"<<cluster_match[i]<<"\n";
    if(match_num==-1)
    {
        j++;
        continue;
    }
		/*
		if(std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>1.1)
		{
			continue;
		}
		*/
    for(int k=0;k<pre_cluster.clst[match_num].pt.size();k++)
    {
        //cloud_temp.y=-(cur_cluster.clst[match_num].pt[k].x*ksize-width/2)*cur_cluster.clst[match_num].pt[k].z/f;
        //cloud_temp.z=((height/2-cur_cluster.clst[match_num].pt[k].y*ksize)*cur_cluster.clst[match_num].pt[k].z)/f+0.4125;
        cloud_temp.y=-(pre_cluster.clst[match_num].pt[k].x*ksize+ksize/2-width/2)*pre_cluster.clst[match_num].pt[k].z/f;
        cloud_temp.z=((height/2-pre_cluster.clst[match_num].pt[k].y*ksize+ksize/2)*pre_cluster.clst[match_num].pt[k].z)/f+0.4125;
        cloud_temp.x=pre_cluster.clst[match_num].pt[k].z;
        cloud_temp.r=colors[i%12][0];
        cloud_temp.g=colors[i%12][1];
        cloud_temp.b=colors[i%12][2];

        //cloud_temp.x+=vel[i].z*1;
        //cloud_temp.y+=-vel[i].x*1;			
       	//cloud_temp.z+=vel[i].y*1;

        cloud_temp.z+=1.5;

        clusted_cloud->points.push_back(cloud_temp);
    }


		
		j++;
	}
	
	clusted_cloud->height=1;
	clusted_cloud->width=clusted_cloud->points.size();
	std::cout<<"cloud->points.size():"<<clusted_cloud->points.size()<<"\n";
	if(!clusted_cloud->points.size())
	{
		return ;
	}
	sensor_msgs::PointCloud2 edit_cloud;
	pcl::toROSMsg (*clusted_cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
	pc_pub.publish(edit_cloud);
}

cv::Mat& estimate_velocity::debug_image(cv::Mat& image)
{
	view_image=image.clone();
	int j = 0;
	uint8_t colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト	


	for(int i=0;i<cur_cluster.clst.size();i++)
	//for(int i=0;i<1;i++)
	{
		//if(i==2)
		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
			for(int u=cur_cluster.clst[i].pt[k].x*ksize;u<cur_cluster.clst[i].pt[k].x*ksize+ksize;u++){
				for(int v=cur_cluster.clst[i].pt[k].y*ksize;v<cur_cluster.clst[i].pt[k].y*ksize+ksize;v++){
					view_image.at<cv::Vec3b>(v,u)[0]=colors[i%12][0];
					view_image.at<cv::Vec3b>(v,u)[1]=colors[i%12][1];
					view_image.at<cv::Vec3b>(v,u)[2]=colors[i%12][2];
				}
			}
		}
	}
	
	for(int k=0;k<match_msg.cur.size();k++)
	{
		
		if(match_msg.cur[k].x==-1)
		{
		  continue;
		}
		int ch=k/width;
		int cw=k%width;
		if(cur_cluster_index[ch][cw]==-1)
		{
		  continue;
		}
		int ph=match_msg.cur[k].y;  
		int pw=match_msg.cur[k].x;
		
		if(pre_cluster_index[ph][pw]==-1)
		{
		  continue;
		}
		int cur_index=cur_cluster_index[ch][cw];//
		int pre_index=pre_cluster_index[ph][pw];//
		//if(cur_index==2)
		//{
			cv::circle(view_image,cv::Point(pw,ph),2,cv::Scalar(0,0,0),-1,CV_AA);
			cv::circle(view_image,cv::Point( (int)(cw),(int)(ch)),2,cv::Scalar(colors[cur_index%12][0],colors[cur_index%12][1],colors[cur_index%12][2]),-1,CV_AA);
		//}
	}
	/*
	for(int i=0;i<match_msg.cur.size();i++){
		if(match_msg.cur[i].x==-1)
		{
			continue;
		}
		cv::circle(view_image,cv::Point( (int)(i%width),(int)(i/width)),2,cv::Scalar(255,255,255),-1,CV_AA);
		cv::circle(view_image,cv::Point(match_msg.cur[i].x,match_msg.cur[i].y),2,cv::Scalar(0,0,0),-1,CV_AA);
	}
	*/
	return view_image;
}
void estimate_velocity::publish_cluster_with_vel(void)
{
	obst_avoid::cluster_with_vel pub_cluster;

	pub_cluster.clst=cur_cluster.clst;
	if(pub_cluster.clst.size()!=cur_cluster.clst.size())
	{
			while(ros::ok())
			{
				std::cout<<"pub_cluster.clst.size()!=cur_cluster.clst.size()\n";
				
			}
	}
	pub_cluster.vel.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		if(pub_cluster.clst[i].pt.size()!=cur_cluster.clst[i].pt.size())
		{
				while(ros::ok())
				{
					std::cout<<"pub_cluster.clst[i].size()!=cur_cluster.clst[i].size())\n";
			
				}
		}
		pub_cluster.vel[i].x=vel[i].x;
		pub_cluster.vel[i].y=vel[i].y;
		pub_cluster.vel[i].z=vel[i].z;
	}
	pub2.publish(pub_cluster);
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"estimate_velocity_class_test");
 
  estimate_velocity est_vel_cls;
  time_class time_cls;
	image_class img_cls;
	img_cls.define_variable();
  while(ros::ok())
	{
		
		img_cls.set_image();

		std::cout<<"4:set_image:"<<time_cls.get_time_now()<<"\n";

    time_cls.set_time();

		std::cout<<"4:set_time:"<<time_cls.get_time_now()<<"\n";

		est_vel_cls.subsuctibe_cluster();

		std::cout<<"4:subsuctibe_cluster:"<<time_cls.get_time_now()<<"\n";

		est_vel_cls.subsuctibe_match_index();

		std::cout<<"4:subsuctibe_match_index:"<<time_cls.get_time_now()<<"\n";

		if(!est_vel_cls.matching_cluster())
		{
			continue;
		}

		std::cout<<"4:matching_cluster:"<<time_cls.get_time_now()<<"\n";
		
		est_vel_cls.estimate_velocity_of_cluster();

		std::cout<<"4:estimate_velocity_of_cluster:"<<time_cls.get_time_now()<<"\n";

		est_vel_cls.draw_velocity(img_cls.get_cur_image_by_ref());

		std::cout<<"4:draw_velocity:"<<time_cls.get_time_now()<<"\n";

		est_vel_cls.publish_pointcloud();

		img_cls.publish_debug_image(est_vel_cls.debug_image(img_cls.get_cur_image_by_ref()) );

		est_vel_cls.publish_cluster_with_vel();

    std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	}

}


