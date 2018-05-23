#include"detect_objects_class.h"

void detect_objects::density_based_clustering(cv::Mat& image)
{

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

	//clustering process
	//std::vector< std::vector<cv::Point2i> > Q;
	Q.clear();
	std::vector<cv::Point2i> Q_p;
	int **cluster_index;

	float sr=0.05;
	int min_pn=3;
	//reserve memory
	Q.reserve(width*height/3);
	Q_p.reserve(width*height/3);
	//resize memory
	cluster_index=new int*[height];
	for(int i=0;i<height;i++){
		cluster_index[i]=new int[width];
	}
	for(int h=0;h<height;h++)//height=height*(2/3)
	{
		for(int w=0;w<width;w++)
		{
			cluster_index[h][w]=-1;
		}
	}
	//density based clustering
	for(int h=height/6;h<height*5/6;h++)//height=height*(2/3)
	{
		for(int w=0;w<width;w++)
		{
			z_temp=depth_image.at<float>(h,w);
			if(z_temp>0.5&&!std::isinf(z_temp))
			{
				x_temp=(w-width/2)*z_temp/f;
				y_temp=(height/2-h)*z_temp/f;
				y_ground=(-a*x_temp-b*y_temp-d)/c;
			  if(y_temp-y_ground>0&&!std::isinf(y_temp)&&y_temp+camera_height<1.5
					&&(image.at<cv::Vec3b>(h,w)[0]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[1]<=255-color_th
					||image.at<cv::Vec3b>(h,w)[2]<=255-color_th))
				{
					//std::cout<<"cluster_index["<<h<<"]["<<w<<"]:"<<cluster_index[h][w]<<"\n";
					if(cluster_index[h][w]==-1)
					{
						cv::Point2i pt;
						pt.x=w;
						pt.y=h;
						Q_p.push_back(pt);
						cluster_index[h][w]=(int)Q.size();
						//int sr_i=(int)(f*sr/z_temp)+1;//search range (int)
						int sr_i=1;
						//std::vector<int> 
						//std::cout<<"Q_p.size():"<<Q_p.size()<<"\n";
						for(int k=0;k<Q_p.size();k++)
						{
							int pn=0;
							int cpn=0;
							std::vector<cv::Point2i> spt;
							spt.resize(sr_i*2*sr_i*2);

							z_temp=depth_image.at<float>(Q_p[k].y,Q_p[k].x);
							x_temp=(Q_p[k].x-width/2)*z_temp/f;
							y_temp=(height/2-Q_p[k].y)*z_temp/f;
							

							for(int j=Q_p[k].y-sr_i;j<=Q_p[k].y+sr_i;j++)
							{
								for(int i=Q_p[k].x-sr_i;i<=Q_p[k].x+sr;i++)
								{
									if(i<0||i>width||j<height/6||j>height*5/6)
									{
										continue;
									}
									else
									{

										float z_s=depth_image.at<float>(j,i);
										if(z_s>0.5&&!std::isinf(z_s))
										{
											float x_s=(i-width/2)*z_s/f;
											float y_s=(height/2-j)*z_s/f;
											float ys_ground=(-a*x_s-b*y_s-d)/c;
											if(y_s-ys_ground>0&&!std::isinf(y_s)&&y_temp+camera_height<1.5
												&&(image.at<cv::Vec3b>(j,i)[0]<=255-color_th
												||image.at<cv::Vec3b>(j,i)[1]<=255-color_th
												||image.at<cv::Vec3b>(j,i)[2]<=255-color_th))
											{			
												//float dist=std::sqrt( std::pow( (z_s-z_temp)*Q_p[k].x+(i-Q_p[k].x)*z_s ,2.0)
											//		+std::pow( (z_s-z_temp)*Q_p[k].y+(j-Q_p[k].y)*z_s ,2.0) )/f;
											//float dist=std::sqrt( std::pow( (z_s-z_temp)*Q_p[k].x+(i-Q_p[k].x)*z_s ,2.0)
											//		+std::pow( (z_s-z_temp)*Q_p[k].y+(j-Q_p[k].y)*z_s ,2.0) + std::pow( (z_s-z_temp) ,2.0) )/f;
												float dist=std::sqrt( std::pow( x_s-x_temp ,2.0)
													+std::pow( y_s-y_temp ,2.0) + std::pow( (z_s-z_temp) ,2.0) );		
												//std::cout<<"x,y,i,j,dist:"<<Q_p[k].x<<","<<Q_p[k].y<<","<<i<<","<<j<<","<<dist<<"\n";
												std::cout<<"dist:"<<dist<<"\n";
												if(dist<sr)
												{
													if(cluster_index[j][i]==-1)
													{
														spt[pn].x=i;
														spt[pn].y=j;
														pn++;
														cpn++;
														//pt.x=i;
														//pt.y=j;
														//Q_p.push_back(pt);													
														//cluster_index[h][w]=(int)Q.size();
													}
													else
													{
														if(cluster_index[j][i]==(int)Q_p.size())
														{
															cpn++;
															continue;
														}
														else
														{

														}
													}
												}
											}
										}						
									}
								}
								//std::cout<<"pn,cpn,min_pn:"<<pn<<","<<cpn<<","<<min_pn<<"\n";
								//density > threshold
								if(cpn>min_pn)
								{
									Q_p.resize(Q_p.size()+pn);
									for(int i=0;i<pn;i++)
									{
										Q_p[Q_p.size()-pn]=spt[i];
										cluster_index[spt[i].y][spt[i].x]=(int)Q.size();
									}
								}
								else
								{

								}

							}//end for k
							Q.push_back(Q_p);
							Q_p.clear();
						}
					}
				}
			}
		}
	}
	std::cout<<"Q.size:"<<Q.size()<<"\n";
	//std::cout<<"selfvoxel: original_size:"<<original_size<<"\n";
}


cv::Mat& detect_objects::draw_cluster(cv::Mat& image){

	temp_image = image.clone();//cv::Mat::zeros(cv::Size(width,height), CV_8UC3);


	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト	


	for(int k=0;k<Q.size();k++)
	{
		for(int kn=0;kn<Q[k].size();kn++)
		{
			temp_image.at<cv::Vec3b>(Q[k][kn].y,Q[k][kn].x)[0]=colors[j%12][0];
			temp_image.at<cv::Vec3b>(Q[k][kn].y,Q[k][kn].x)[1]=colors[j%12][1];
			temp_image.at<cv::Vec3b>(Q[k][kn].y,Q[k][kn].x)[2]=colors[j%12][2];
		}
		j++;
	}
	std::cout<<"return temp_image\n";
	return temp_image;
}
