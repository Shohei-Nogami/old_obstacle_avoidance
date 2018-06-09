#include"culculate_centroid_class.h"

culculate_centroid::culculate_centroid()
	:it_pub1(nh_pub1)
{	
	nh_cluster.setCallbackQueue(&queue_cluster);
	sub_cluster=nh_cluster.subscribe("/cluster",1,&culculate_centroid::cluster_callback,this);
	//nh_optflw.setCallbackQueue(&queue_optflw);
	//sub_optflw=nh_optflw.subscribe("objects_velocity",1,&culculate_centroid::opticalflow_callback,this);
	
	nh_match.setCallbackQueue(&queue_match);
	sub_match=nh_match.subscribe("cluster_matching_index",1,&culculate_centroid::match_index_callback,this);

  pc_pub = nh_pubpcl.advertise<sensor_msgs::PointCloud2>("clusted_cloud", 1);
	pub1=it_pub1.advertise("vel_image",1);//test string

	pub2 = nh_pub2.advertise<obst_avoid::cluster_with_vel>("cluster_with_vel", 1);
	pub3 = nh_pub3.advertise<obst_avoid::objects_info>("objects_info", 1);
	

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
culculate_centroid::~culculate_centroid()
{


}
void culculate_centroid::subsuctibe_cluster(void)
{
	queue_cluster.callOne(ros::WallDuration(1));
}
void culculate_centroid::cluster_callback(const obst_avoid::cluster::ConstPtr& msg)
{
	cur_cluster.dX=msg->dX;
	cur_cluster.t=msg->t;
	cur_cluster.clst=msg->clst;
}
void culculate_centroid::subsuctibe_match_index(void){
	queue_match.callOne(ros::WallDuration(1));
}

void culculate_centroid::match_index_callback(const obst_avoid::matching::ConstPtr& msg)
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
bool culculate_centroid::matching_cluster(void)
{
	//std::cout<<"aa\n";
	if(!match_msg.pre.size()||!cur_cluster.clst.size())
	{
		return false;
	}

	set_gp();

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
	if(!pre_gp.size())
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
//matching value by gp (distance)
	match_gp.resize((int)cur_cluster.clst.size());

	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		match_gp[i].resize((int)pre_cluster.clst.size());
	}
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		for(int k=0;k<pre_cluster.clst.size();k++)
		{
			culc_distance_3f(cur_gp[i],pre_gp[k],match_gp[i][k]);
		}
	}

//init match value 

	matched.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		matched[i]=false;
		cluster_match[i]=-1;
	}
//matching cluster

	for(int k=0;k<pre_cluster.clst.size();k++)
	{
	//min distance cluster and max matching points
		//min distance
		float min_dis=5;//match_gp[0][k];
		int min_dis_i=-1;//0;
		//max matching point
		int max_pt=0;//match_n[0][k];
		int max_pt_i=-1;//0;

		//min dif size
		
		double min_dsize=5*5;
		//min_dsize=std::abs(pre_cluster_size[k]-cur_cluster_size[0]);
		int min_dsize_i=-1;//0;

		//ef
		float ef=0;
		int ef_i=-1;

		for(int i=0;i<cur_cluster.clst.size();i++)
		{
			if(matched[i])
			{
				continue;
			}
			//min distance
			if(min_dis>match_gp[i][k])
			{
				min_dis=match_gp[i][k];
				min_dis_i=i;
			}
			//max num of point 
			if(max_pt<match_n[i][k])
			{
				max_pt=match_n[i][k];
				max_pt_i=i;
			}
			/*
			//min dif of size
			float dif_size = std::abs(pre_cluster_size[k]-cur_cluster_size[i])
			if(min_dsize > dif_size)
			{
				min_dsize=dif_size;
				min_dsize_i=i;
			}
			*/
			float ef_temp=match_n[i][k] + (1/match_gp[i][k]);
			if(ef<ef_temp)
			{
				ef=ef_temp;
				ef_i=i;
			}
			
		}
		/*
		if(max_pt_i==0)
		{
			float dX;
			culc_distance_3f(cur_gp[min_dis_i],pre_gp[k],dX);

			//float dif_size = std::abs(pre_cluster_size[k]-cur_cluster_size[min_dis_i]);

			cluster_match[min_dis_i]=k;
			matched[min_dis_i]=true;
		}
		else if(min_dis_i==max_pt_i)//good match 
		{
			cluster_match[max_pt_i]=k;
			matched[max_pt_i]=true;
		}
		else
		{
			cluster_match[max_pt_i]=k;
			matched[max_pt_i]=true;
		}
		*/
		cluster_match[ef_i]=k;
		matched[ef_i]=true;
		//std::cout<<"1111\n";
	}
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

void culculate_centroid::set_gp(void)
{
	std::cout<<"1\n";
	/*
	if(cur_cluster_size.size())
	{
		//pre_cluster_size.clear();
		pre_cluster_size.resize(cur_cluster_size.size());
		for(int i=0;i<cur_cluster.clst.size();i++)
		{
			 pre_cluster_size[i]=cur_cluster_size[i];
		}
	}
	*/
	std::cout<<"1.1\n";
	//std::cout<<"cur_cluster.clst.size()):"<<cur_cluster.clst.size()<<"\n";
	//cur_cluster_size.clear();
	cur_cluster_size.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		 cur_cluster_size[i]=0;
	}
	cur_cluster_radius.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster_radius.size();i++)
	{
		 cur_cluster_radius[i]=0;
	}
	if(cur_gp.size())
	{
		//pre_gp.clear();
		pre_gp.resize(cur_gp.size());
		for(int i=0;i<cur_gp.size();i++)
		{
			 pre_gp[i]=cur_gp[i];
		}
	}	

	std::cout<<"1.2\n";

	//cur_gp.clear();
	cur_gp.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		cur_gp[i].x=0;
		cur_gp[i].y=0;
		cur_gp[i].z=0;

		//add0531
		float x_min,x_max;
		float z_min,z_max;
		float x_tmp,z_tmp;
			x_min=(cur_cluster.clst[i].pt[0].x*ksize+ksize/2-width/2)*cur_cluster.clst[i].pt[0].z/f;
		
		x_max=x_min;
		
		z_min=cur_cluster.clst[i].pt[0].z;
		z_max=z_min;
		
		
		for(int k=1;k<cur_cluster.clst[i].pt.size();k++)
		{
		  		
			x_tmp=(cur_cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cur_cluster.clst[i].pt[k].z/f;
			//cur_gp[i].y+=((height/2-cur_cluster.clst[i].pt[k].y*ksize+ksize/2)*cur_cluster.clst[i].z)/f+0.4125;
			z_tmp=cur_cluster.clst[i].pt[k].z;
		  if(x_max<x_tmp)
		  {
		    x_max=x_tmp;
		  }
		  if(x_min>x_tmp)
		  {
		    x_min=x_tmp;
		  }
		  if(z_max<z_tmp)
		  {
		    z_max=z_tmp;
		  }
		  if(z_min>z_tmp)
		  {
		    z_min=z_tmp;
		  }
		  
		}
		//end
		//radius
		cur_cluster_radius[i]=std::sqrt((z_max-z_min)*(z_max-z_min) + (x_max-x_min)*(x_max-x_min) )/2.0;
		
		//
		float rate=0.2;
		if(x_max-x_min>0.1)
		{
		  x_max-=(x_max-x_min)*rate;
		  x_min+=(x_max-x_min)*rate;
		}
		if(z_max-z_min>0.1)
		{
		  z_max-=(z_max-z_min)*rate;
		  z_min+=(z_max-z_min)*rate;
		}
		int count=0;
		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
		  float x=(cur_cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cur_cluster.clst[i].pt[k].z/f;
		  float z=cur_cluster.clst[i].pt[k].z;
			//add cluster size
			cur_cluster_size[i]+=std::pow(ksize*cur_cluster.clst[i].pt[k].z/f,2.0);
		  
		  if(x<x_min||x>x_max
		    ||z<z_min||z>z_max)
		  {
		    continue;
		  }
			//cur_gp[i].x+=(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
			////cur_gp[i].y+=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].z)/f+0.4125;
			//cur_gp[i].z+=cur_cluster.clst[i].pt[k].z;
			cur_gp[i].x+=x;
			//cur_gp[i].y+=((height/2-cur_cluster.clst[i].pt[k].y*ksize+ksize/2)*cur_cluster.clst[i].z)/f+0.4125;
			cur_gp[i].z+=z;
			count++;

		}
		cur_gp[i].x=cur_gp[i].x/count;
		//cur_gp[i].y=cur_gp[i].y/(int)cur_cluster.clst[i].pt.size();
		cur_gp[i].z=cur_gp[i].z/count;

		
	}

}
bool culculate_centroid::culculate_centroid_of_cluster(void)
{
	/*
	cur_cluster_size.resize(cur_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		 cur_cluster_size[i]=0;
	}
	*/
	
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

	if(!pre_gp.size())
	{
		return false;
	}
	float dX;
	cv::Point3f temp0=cv::Point3f(0,0,0);
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
		culc_distance_3f(vel[i],temp0,dX);
		if(dX>1.1)
		{
			if(track_n[i]>0)
			{
				vel[i] = pre_vel[i]; 
			}
			else
			{
				vel[i].x = 0;
				vel[i].y = 0;
				vel[i].z = 0;
			}
		}
		int k=0;
		float T=cur_cluster.t/5;
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

void culculate_centroid::predict_cluster(void)
{

}

void culculate_centroid::draw_velocity(cv::Mat& image)
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

void culculate_centroid::publish_pointcloud(void)
{
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	clusted_cloud->points.clear();
	clusted_cloud->points.reserve(width*height);

	pcl::PointXYZRGB cloud_temp;

	for(int i=0;i<cur_cluster.clst.size();i++)
	{

		/*
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
		*/
		for(int t=0;t<4;t++)
		{
			if(track_n[i]<1||std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>1.1
				||cur_cluster_size[i]>1.0*1.0
				||cur_cluster_size[i]<0.2*0.2
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
	/*
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
	*/

		
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

cv::Mat& culculate_centroid::debug_image(cv::Mat& image)
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
void culculate_centroid::publish_cluster_with_vel(void)
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

void culculate_centroid::publish_objects_info(void)
{
	
	obst_avoid::objects_info objs;
	//obst_avoid::object_info obj;

	objs.obj.resize(cur_cluster.clst.size());
	for(int i=0;i<objs.obj.size();i++)
	{
		objs.obj[i].pos.x=cur_gp[i].x;
		objs.obj[i].pos.y=cur_gp[i].y;
		objs.obj[i].pos.z=cur_gp[i].z;

		objs.obj[i].size=cur_cluster_size[i];
		objs.obj[i].r=cur_cluster_radius[i];
		if(pre_gp.size())
		{
			objs.obj[i].match=cluster_match[i];
		}
		else
		{
			objs.obj[i].match=-1;
		}
		/*
		if(objs.obj[i].match > (int)pre_cluster.clst.size())
		{
			int a;
			std::cout<<"objs.obj[i].match > pre_cluster.size()\n";
			std::cout<<"objs.obj["<<i<<"].match"<<objs.obj[i].match<<"\n";
			std::cout<<"(int)pre_cluster.clst.size():"<<(int)pre_cluster.clst.size()<<"\n";
			std::cout<<"cluster_match["<<i<<"]:"<<cluster_match[i]<<"\n";
			std::cout<<"pre_cluster_match["<<cluster_match[i]<<"]:"<<pre_cluster_match[cluster_match[i]]<<"\n";
			std::cin>>a;
		}
		std::cout<<"(int)pre_cluster.clst.size():"<<(int)pre_cluster.clst.size()<<"\n";
		std::cout<<"cluster_match["<<i<<"]:"<<cluster_match[i]<<"\n";
		std::cout<<"pre_cluster_match["<<cluster_match[i]<<"]:"<<pre_cluster_match[cluster_match[i]]<<"\n";
		*/

		objs.obj[i].pt.resize(cur_cluster.clst[i].pt.size());
		for (int k = 0; k < objs.obj[i].pt.size(); k++)
		{
			objs.obj[i].pt[k] = cur_cluster.clst[i].pt[k];
		}


	}
	objs.dt=cur_cluster.t;
	std::cout<<"cur_cluster.t:"<<cur_cluster.t<<"\n";
	std::cout<<"objs.dt:"<<objs.dt<<"\n";
	objs.dX=cur_cluster.dX;

	pub3.publish(objs);
}

void culculate_centroid::culc_distance_3f(const cv::Point3f x1,cv::Point3f x2,float& dis)
{
	dis = std::sqrt((x1.x-x2.x)*(x1.x-x2.x) + (x1.y-x2.y)*(x1.y-x2.y) + (x1.z-x2.z)*(x1.z-x2.z) );
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"culculate_centroid_class_test");
 
  culculate_centroid clc_cnt_cls;
  time_class time_cls;
	image_class img_cls;
	img_cls.define_variable();
  while(ros::ok())
	{
		
		img_cls.set_image();

		std::cout<<"4:set_image:"<<time_cls.get_time_now()<<"\n";

    time_cls.set_time();

		std::cout<<"4:set_time:"<<time_cls.get_time_now()<<"\n";

		clc_cnt_cls.subsuctibe_cluster();

		std::cout<<"4:subsuctibe_cluster:"<<time_cls.get_time_now()<<"\n";

		clc_cnt_cls.subsuctibe_match_index();

		std::cout<<"4:subsuctibe_match_index:"<<time_cls.get_time_now()<<"\n";

		if(!clc_cnt_cls.matching_cluster())
		{
			continue;
		}

		std::cout<<"4:matching_cluster:"<<time_cls.get_time_now()<<"\n";
		
		clc_cnt_cls.culculate_centroid_of_cluster();

		std::cout<<"4:culculate_centroid_of_cluster:"<<time_cls.get_time_now()<<"\n";

		clc_cnt_cls.draw_velocity(img_cls.get_cur_image_by_ref());

		std::cout<<"4:draw_velocity:"<<time_cls.get_time_now()<<"\n";

		clc_cnt_cls.publish_pointcloud();

		img_cls.publish_debug_image(clc_cnt_cls.debug_image(img_cls.get_cur_image_by_ref()) );

		clc_cnt_cls.publish_cluster_with_vel();

		clc_cnt_cls.publish_objects_info();

    std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	}

}



