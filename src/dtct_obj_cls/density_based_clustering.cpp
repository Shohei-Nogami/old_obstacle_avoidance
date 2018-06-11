#include"detect_objects_class.h"

//void detect_objects::density_based_clustering
void detect_objects::filter_process(void){
	//if(ksize==1)
	//	return ;
	std::vector<float> depth_median;
	depth_median.reserve((median_param/2+1)*(median_param/2+1));
	filted_image=cv::Mat::zeros(cv::Size(width,height), CV_32FC1);
	std::cout<<"aaa\n";
	for(int h=0;h<height/ksize;h++){
		for(int w=0;w<width/ksize;w++){
			for(int l=-median_param/2;l<=median_param/2;l++)
			{
				for(int m=-median_param/2;m<=median_param/2;m++)
				{
					if(std::isnan(depth_image.at<float>(h*ksize+l,w*ksize+m))
						||std::isinf(depth_image.at<float>(h*ksize+l,w*ksize+m)) 
						||h+l<0||height/ksize<=h+l
						||w+m<0||width/ksize<=w+m )
					{
						continue;
					}
					depth_median.push_back(depth_image.at<float>(h*ksize+l,w*ksize+m) );
				}
			}
			std::sort(depth_median.begin(),depth_median.end());
			if((int)depth_median.size())
			{
				filted_image.at<float>(h,w)=depth_median[(int)depth_median.size()/2];
			}
			else{
				filted_image.at<float>(h,w)=0;						
			}
			depth_median.clear();
		}
	}
}
void detect_objects::density_based_clustering(cv::Mat& image)
{
	float floor_th=0.2;
	float camera_height=0.4125;
	float camera_bias=0.2;
	const float y_th=0.40;
	const float cam_y=0.4125;
	float z_temp,x_temp,y_temp;
	float a,b,c,d;
	
	pcl::PointXYZ voxel_element_temp;

	//std::cout<<"aa\n";
	//ground estimate
	ground_estimation_from_image(y_th,cam_y,a,b,c,d);

	float y_ground;
	uint8_t color_th=3;

	//if(std::abs(d-(camera_height+camera_bias))>=0.15){
	if(1){
		a=-0.08;
		b=0;
		c=1;
		d=(camera_height+camera_bias)-floor_th;
	}
	else{
		d-=floor_th;
	}
	std::cout<<a<<" x + "<<b<<" y + "<<c<<" z + "<<d<<" = 0\n";

	//clustering process
	//std::vector< std::vector<cv::Point2i> > Q;
	for(int i=0;i<Q.clst.size();i++){
		Q.clst[i].pt.clear();
	}
	Q.clst.clear();
	//std::vector<cv::Point2i> Q_p;
	obst_avoid::cluster_element Q_p;
	int **cluster_index;

	float sr=0.1;
	int min_pn=6;
	//reserve memory
	Q.clst.reserve(width*height);
	Q_p.pt.reserve(width*height);
	//resize memory
	//int ksize=1;
	int search_range=1;
	double depth_threshold=0.03;//0.02;
	int min_pn0=8;
	double eps=0.02;
	bool searched_flag[height/ksize][width/ksize];
	//std::vector<cv::Point2i> task_objects;
	//std::vector<obst_avoid::point2i> task_objects;
	obst_avoid::cluster_element task_objects;
	task_objects.pt.reserve(height*width);
	//initialize searched_flag
	for(int h=0;h<height/ksize;h++){
		for(int w=0;w<width/ksize;w++){
			searched_flag[h][w]=false;
		}
	}
	
	std::cout<<"1\n";
	float height_th=1.0;//1.5;
	//cv::Point2i temp;
	obst_avoid::cluster_point temp;
	for(int h=0+search_range;h<height/ksize-search_range;h++){
		for(int w=0+search_range;w<width/ksize-search_range;w++){
			double depth_0=filted_image.at<float>(h,w);
			if(searched_flag[h][w]||std::isnan(depth_0)||depth_0==0){
				continue;
			}
			double y_0=(-(h*ksize-height/2))*depth_0/f;
			double x_0=(w*ksize-width/2)*depth_0/f;
			double y_ground=(-a*depth_0-b*(-x_0)-d)/c;
			if(y_0-y_ground<=0||y_0-y_ground>height_th-camera_height){
			//if(y_0-camera_height<=0||y_0>1.5-camera_height){
				continue;
			}
			double objects_depth=0;
			Q_p.pt.clear();
			temp.x=w;
			temp.y=h;
			temp.z=depth_0;
			task_objects.pt.push_back(temp);
			Q_p.pt.push_back(temp);
			searched_flag[h][w]=true;
			//search process
			for(int i=0;i<task_objects.pt.size();i++){
				//std::vector<cv::Point2i> task_pt;
				obst_avoid::cluster_element task_pt;
				int pt_n=0;
				task_pt.pt.reserve(search_range*search_range);
				float depth_0=filted_image.at<float>(task_objects.pt[i].y,task_objects.pt[i].x);
				//std::cout<<"depth_0:"<<depth_0<<"\n";
				//std::cout<<"task_objects.pt[i]:"<<task_objects.pt[i]<<"\n";
				if(depth_0>1)
				{
					//min_pn=depth_0*(-1)+8;
					min_pn=depth_0*(-1)+min_pn0;
				}
				else
				{
					min_pn=min_pn0;
				}
				for(int l=-search_range;l<=search_range;l++){
					for(int m=-search_range;m<=search_range;m++){
						if(searched_flag[task_objects.pt[i].y+l][task_objects.pt[i].x+m])
						{
							pt_n++;
							continue;
						}
						if(0 > task_objects.pt[i].y+l || task_objects.pt[i].y+l > height/ksize
							||0 > task_objects.pt[i].x+m || task_objects.pt[i].x+m > width/ksize)
						{
							continue;
						}
						//std::cout<<"("<<task_objects.pt[i].y+l<<","<<task_objects.pt[i].x+m<<")\n";
						float depth_i=filted_image.at<float>(task_objects.pt[i].y+l,task_objects.pt[i].x+m);
						//std::cout<<"depth_i:"<<depth_i<<"\n";
						if(std::isnan(depth_i)||depth_i==0)
							continue;
						float y_i=(-((task_objects.pt[i].y+l)*ksize-height/2))*depth_i/f;
						float x_i=((task_objects.pt[i].x+m)*ksize-width/2)*depth_i/f;
						double y_ground=(-a*depth_i-b*(-x_i)-d)/c;
						if(y_i-y_ground<=0||y_i>height_th-camera_height)
						//if(y_0-camera_height<=0||y_i>1.5-camera_height)
						{
							continue;
						}


						//std::cout<<"x_i-x_0:"<<x_i-x_0<<"\n";
						if(std::abs(depth_i-depth_0)<depth_threshold){
						//if(std::abs(depth_i-depth_0)<eps){
						//if(std::sqrt(std::pow(depth_i-depth_0,2.0)+std::pow(x_i-x_0,2.0))<eps){
						//	searched_flag[task_objects.pt[i].y+l][task_objects.pt[i].x+m]=true;
							temp.x=task_objects.pt[i].x+m;
							temp.y=task_objects.pt[i].y+l;
							temp.z=depth_i;
						//	task_objects.pt.push_back(temp);
							task_pt.pt.push_back(temp);
							//-----
							/*
							for(int u=temp.x*ksize;u<temp.x*ksize+ksize;u++){
								for(int v=temp.y*ksize;v<temp.y*ksize+ksize;v++){
							//		int u=temp.x*ksize;
							//		int v=temp.y*ksize;
									int j=(int)objects.rect.size();
									detectd_image.at<cv::Vec3b>(v,u)[0]=colors[j%12][0];
									detectd_image.at<cv::Vec3b>(v,u)[1]=colors[j%12][1];
									detectd_image.at<cv::Vec3b>(v,u)[2]=colors[j%12][2];
								}
							}
							*/
							//-----
						}
					}//m
				}//l
				if(min_pn<((int)task_pt.pt.size()+pt_n))
				{
					Q_p.pt.push_back(task_objects.pt[i]);
					task_objects.pt.insert(task_objects.pt.end(),task_pt.pt.begin(),task_pt.pt.end());
					for(int i=0;i<task_pt.pt.size();i++)
					{
						searched_flag[task_pt.pt[i].y][task_pt.pt[i].x]=true;
						
					}
				}
			}//task
			Q.clst.push_back(Q_p);
			task_objects.pt.clear();
		}
	}
	std::cout<<"Q.clst.size:"<<Q.clst.size()<<"\n";
	//std::cout<<"selfvoxel: original_size:"<<original_size<<"\n";
}

cv::Mat& detect_objects::draw_cluster(cv::Mat& image)
{

	if(!Q.clst.size())
	{
		std::cout<<"!Q.clst.size()\n";
		return image;
	}
	temp_image = image.clone();//cv::Mat::zeros(cv::Size(width,height), CV_8UC3);
	//temp_image = cv::Mat::zeros(cv::Size(width,height), CV_8UC3);
	
	
	int j = 0;
	uint8_t colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト	


	for(int k=0;k<Q.clst.size();k++)
	{
		//if((int)Q.clst[k].pt.size()<10)
		//{
		//	continue;
		//}
		for(int kn=0;kn<Q.clst[k].pt.size();kn++)
		{
			for(int u=Q.clst[k].pt[kn].x*ksize;u<Q.clst[k].pt[kn].x*ksize+ksize;u++){
				for(int v=Q.clst[k].pt[kn].y*ksize;v<Q.clst[k].pt[kn].y*ksize+ksize;v++){
					temp_image.at<cv::Vec3b>(v,u)[0]=colors[j%12][0];
					temp_image.at<cv::Vec3b>(v,u)[1]=colors[j%12][1];
					temp_image.at<cv::Vec3b>(v,u)[2]=colors[j%12][2];
				}
			}
		}
		j++;
	}
	std::cout<<"return temp_image\n";
	return temp_image;
}
void detect_objects::publish_cluster(double& v,double& w,double& dt)
{
	obst_avoid::cluster pub_Q;
	pub_Q.dX.z=v*cos(w*dt)*dt;
	pub_Q.dX.x=v*sin(w*dt)*dt;
	pub_Q.dX.y=0;
	pub_Q.t=dt;
	pub_Q.clst.reserve(Q.clst.size());
	for(int i=0;i<Q.clst.size();i++)
	{
		if((int)Q.clst[i].pt.size()<10)
		{
			continue;
		}
		pub_Q.clst.push_back(Q.clst[i]);
	}
	if(!Q.clst.size())
	{	
		std::cout<<"!Q.clst.size()\n";
		return;
	}
	pub_cluster.publish(pub_Q);
}

