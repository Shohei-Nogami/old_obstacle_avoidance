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

  pc_pub = nh_pubpcl.advertise<sensor_msgs::PointCloud2>("clusterd_cloud", 1);
	pub1=it_pub1.advertise("vel_image",1);//test string

	//--reserve memory
	//index of maching
	match_msg.pre.reserve(width*height);
	match_msg.cur.reserve(width*height);
	cluster_match.reserve(width*height);

	//--resize data
	pre_cluster_index.resize(height);
	for(int h=0;h<height;h++)
	{
		pre_cluster_index[h].resize(width);
	}
	cur_cluster_index.resize(height);
	for(int h=0;h<height;h++)
	{
		cur_cluster_index[h].resize(width);
	}
	std::cout<<"cur_cluster_index.size():"<<cur_cluster_index.size()<<"\n";
	std::cout<<"cur_cluster_index[0].size():"<<cur_cluster_index[0].size()<<"\n";
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
	std::cout<<"aa\n";
	if(!match_msg.pre.size()||!cur_cluster.clst.size())
	{
		return false;
	}
	cluster_match.clear();
	cluster_match.resize(cur_cluster.clst.size());

	pre_cluster=cur_cluster;
	/*
for(int i=0;i<pre_cluster.clst.size();i++)
	{
		pre_cluster.clst[i].pt.resize(cur_cluster.clst[i].pt.size());
		//std::copy(pre_cluster.clst[i].pt.begin(),pre_cluster.clst[i].pt.end(),cur_cluster.clst[i].pt.begin());
		std::copy(pre_cluster.clst[i].pt.begin(),pre_cluster.clst[i].pt.end(),std::back_inserter(cur_cluster.clst[i].pt));
	}
	pre_cluster.dX.x=cur_cluster.dX.x;
	pre_cluster.dX.y=cur_cluster.dX.y;
	pre_cluster.dX.z=cur_cluster.dX.z;
	pre_cluster.t=cur_cluster.t;
	*/
	std::cout<<"aaaa\n";
	if(cur_cluster_index.size())
	{
	//	pre_cluster_index=cur_cluster_index;

		pre_cluster_index.resize(cur_cluster_index.size());
		std::copy(pre_cluster_index.begin(),pre_cluster_index.end(),cur_cluster_index.begin());
		
	}
	std::cout<<"aaaaaa\n";
	//cluster number in cluster_index
	//init index 
	for(int h=0;h<height;h++)
	{
		for(int w=0;w<width;w++)
		{
			cur_cluster_index[h][w]=-1;
		}
	}
	std::cout<<"aabb\n";
	//write index
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
			int h=cur_cluster.clst[i].pt[k].y;
			int w=cur_cluster.clst[i].pt[k].x;
			for(int v=h*ksize;v<h*ksize+ksize;v++)
			{
				for(int u=h*ksize;u<h*ksize+ksize;u++)
				{
					cur_cluster_index[v][u]=i;
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
	std::vector<int> match_n;
//	int *match_n;
//	match_n=new int[(int)pre_cluster.clst.size()];
	//int match_n[(int)pre_cluster.clst.size()];
//	match_n.clear();
//	match_n.reserve(pre_cluster.clst.size());
	
	match_n.resize((int)pre_cluster.clst.size());
	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		//std::cout<<"1\n";
		//init match_n
		for(int k=0;k<pre_cluster.clst.size();k++)
		{
			match_n[k]=0;
			//match_n.push_back(0);
		}
		//return false; 
		//std::cout<<"11\n";
		int ph,pw,ch,cw,clst_n;
		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
			// point of previous cluster 
			ph=cur_cluster.clst[i].pt[k].y*ksize;
			pw=cur_cluster.clst[i].pt[k].x*ksize;
			//std::cout<<"p h,w:"<<ph<<","<<pw<<"\n";
			//std::cout<<"match_msg.cur.size():"<<match_msg.cur.size()<<"\n";
			// match point from previous cluster 
			ch=match_msg.cur[ph*width+pw].y;
			cw=match_msg.cur[ph*width+pw].y;
			//std::cout<<"c h,w:"<<ch<<","<<cw<<"\n";
			if(ch==-1||cw==-1)
			{
				continue;
			}
			// match point of current cluster 
			clst_n=pre_cluster_index[ch][cw];
			if(clst_n==-1)
			{
				continue;
			}
			//std::cout<<"clst_n:"<<clst_n<<"\n";
			std::cout<<"match_n.size:"<<match_n.size()<<"\n";
			std::cout<<"pre_cluster.clst.size():"<<pre_cluster.clst.size()<<"\n";
			std::cout<<"match_n["<<clst_n<<"]:"<<match_n[clst_n]<<"\n";
			match_n[clst_n]++;
		}
		//std::cout<<"111\n";
		//select best match
		int max_n=0;
		int max_i=-1;
		for(int k=0;k<pre_cluster.clst.size();k++)
		{
			if(max_n<match_n[k])
			{
				max_n=match_n[k];
				max_i=k;
			}
		}
		if(max_i!=-1)
		{
			cluster_match[i]=max_i;//cur -> pre
		}
		else
		{
			cluster_match[i]=-1;
		}
		//std::cout<<"1111\n";
	}
	std::cout<<"end\n";
	//delete match_n;
	return true;
}

bool estimate_velocity::estimate_velocity_of_cluster(void)
{
	if(cur_gp.size())
	{
    pre_gp=cur_gp;
	}	
	cur_gp.clear();
	cur_gp.resize(cur_cluster.clst.size());
	vel.clear();
	vel.resize(cur_cluster.clst.size());

	for(int i=0;i<cur_cluster.clst.size();i++)
	{
		cur_gp[i].x=0;
		cur_gp[i].y=0;
		cur_gp[i].z=0;

		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
			cur_gp[i].x+=(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
			//cur_gp[i].y+=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].z)/f+0.4125;
			cur_gp[i].z+=cur_cluster.clst[i].pt[k].z;
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
		//std::cout<<"cluster_match["<<i<<"]:"<<cluster_match[i]<<"\n";
		if(match_num==-1)
		{
			continue;
		}
		vel[i].x = (cur_gp[i].x-pre_gp[match_num].x)/cur_cluster.t;
		vel[i].y = 0;//(cur_gp[i].y-pre_gp[i].y)/cur_cluster.t;
		vel[i].z = (cur_gp[i].z-pre_gp[match_num].z)/cur_cluster.t;
		//std::cout<<"cur_gp["<<i<<"]:"<<cur_gp[i]<<"\n";
		//std::cout<<"pre_gp["<<i<<"]:"<<pre_gp[i]<<"\n";
		//std::cout<<"cur_cluster.t:"<<cur_cluster.t<<"\n";
		std::cout<<"vel["<<i<<"]:"<<vel[i]<<"\n";
	}
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
		if(!std::isnan(vel[i].x)
				&&std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>0.1
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
				gp.x+=cur_cluster.clst[i].pt[k].x*ksize;
				gp.y+=cur_cluster.clst[i].pt[k].y*ksize;

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
		for(int k=0;k<cur_cluster.clst[i].pt.size();k++)
		{
			cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
			cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
			cloud_temp.x=cur_cluster.clst[i].pt[k].z;
			cloud_temp.r=colors[j%12][0];
			cloud_temp.g=colors[j%12][1];
			cloud_temp.b=colors[j%12][2];

		
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

    std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	}

}


