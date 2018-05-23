#include"detect_objects_class.h"

void detect_objects::subscribe_opticalflow(void){
	queue_optflw.callOne(ros::WallDuration(1));
}
void detect_objects::opticalflow_callback(const obst_avoid::vel3d::ConstPtr& msg){
	vX.pt=msg->pt;
	vX.vel=msg->vel;
}
void detect_objects::add_velocity_to_cluster(void){
	if(!vX.pt.size())
	{
		return ;
	}
	std::cout<<"vX.pt,vel:"<<vX.pt.size()<<","<<vX.vel.size()<<"\n";
	//std::vector< std::vector<pcl::PointXYZ> > cluster_vel_elm;
	pcl::PointXYZ vel_element;
	cluster_vel_elm.resize(cluster.size());
	int h,w;
	int nx,nz,ny;
	int cn;
	//std::cout<<"vX.pt[0],vel[0]:"<<vX.pt[0]<<","<<vX.vel[0]<<"\n";
	for(int k=0;k<vX.pt.size();k++){
		//std::cout<<"for \n";
		h=vX.pt[k].h;
		w=vX.pt[k].w;
		//std::cout<<",h,w:"<<h<<","<<w<<"\n";
		nx=index_vxl[h][w].nx;
		ny=index_vxl[h][w].ny;
		nz=index_vxl[h][w].nz;
		//std::cout<<"nx,ny,nz,h,w,cn:"<<nx<<","<<ny<<","<<nz<<","<<h<<","<<w<<",";
	
		if(nz!=-1)
		{
			//std::cout<<"nx,ny,nz:"<<nx<<","<<ny<<","<<nz<<"\n";
			cn=clusted_index[nz][nx][ny];
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
	//std::cout<<"cluster.size():"<<cluster.size()<<"\n";
	/*
	for(int i=0;i<cluster.size();i++){
		std::cout<<"cluster_vel_elm["<<i<<"]:"<<cluster_vel_elm[i].size()<<"\n";
		
	}
	*/
}

void detect_objects::estimate_velocity_of_cluster(void)
{
	//std::vector<pcl::PointXYZ> cluster_vel;
	cluster_vel.resize(cluster.size());
	for(int i=0;i<cluster.size();i++)
	{
		int n=0;
		cluster_vel[i].x=0;
		cluster_vel[i].y=0;
		cluster_vel[i].z=0;
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
		cluster_vel[i].x=cluster_vel[i].x/n;
//		cluster_vel.y=cluster_vel.y/n;
		cluster_vel[i].z=cluster_vel[i].z/n;
		
	}
	for(int i=0;i<cluster.size();i++)
	{
		if(!std::isnan(cluster_vel[i].x))
		{
			std::cout<<"cluster(num,speed(x,z)):("<<i<<",speed("<<cluster_vel[i].x<<","<<cluster_vel[i].z<<"))\n";//when speed is nan, cluster_vel[i].size() is 0
		}
	}
}
void detect_objects::draw_velocity(cv::Mat& image)
{
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	view_vel_image=image.clone();
	for(int i=0;i<cluster.size();i++)
	{
		float volume_threshold=0.1*0.1*0.1;
		float one_point_volume=voxel_size_x*voxel_size_z*voxel_size_y;
		if(one_point_volume*(int)cluster[i].size()>volume_threshold&&!std::isnan(cluster_vel[i].x)&&
				std::sqrt(std::pow(cluster_vel[i].x,2.0)+std::pow(cluster_vel[i].z,2.0))>0.1)
		{
			std::string vel_string_x,vel_string_z,vel_string;
			vel_string_x=std::to_string(cluster_vel[i].x);//(int)(cluster_vel[i].x*1000)/(float)1000);
			vel_string_z=std::to_string(cluster_vel[i].z);//(int)(cluster_vel[i].z*1000)/(float)1000);
			vel_string="("+vel_string_x.substr(0,5)+","+vel_string_z.substr(0,5)+")";
			cv::Point2i gp;
			gp.x=0;
			gp.y=0;
			for(int k=0;k<cluster[i].size();k++)
			{
				gp.x+=-cluster[i][k].y/cluster[i][k].x*f;
				gp.y+=(cluster[i][k].z-0.4125)/cluster[i][k].x*f;

				//view_vel_image.at<cv::Vec3b>(gp.y,gp.x)
						
			}
			gp.x=(int)(gp.x/(int)cluster[i].size())+width/2-100;
			gp.y=height/2-(int)(gp.y/(int)cluster[i].size());
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


