#include"vfh+_class.h"

vfh_class::vfh_class()
	:it_pub(nh_pub),it_pub2(nh_pub2),grid_resolution(201),grid_size(12.0),EXECUTED_CALLBACK(false),binary_threshold(90)
{
	pub=it_pub.advertise("grid_image",1);
	pub2=it_pub2.advertise("binary_grid_image",1);
	pc_pub = nh_pubpcl.advertise<sensor_msgs::PointCloud2>("cluster_usedvfh", 1);
	pub_wheel=nh_pub_w.advertise<obst_avoid::wheel_msg>("vel_data",1);

	nh_sub.setCallbackQueue(&queue);
//	sub=nh_sub.subscribe("/zed/point_cloud/cloud_registered",1,&vfh_class::plc_callback,this);
	sub=nh_sub.subscribe("/cluster_with_vel",1,&vfh_class::cluster_callback,this);
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC1);
	grid_map=m.clone();
	cv::Mat m_view = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC3);
	grid_map_view=m_view.clone();
	binary_grid_map_view=m_view.clone();
	grid_cell_size=grid_size/grid_resolution;

//set parameter of trajectory
	max_process_n.reserve(vfh_resolution);
	rank_trajectory.resize(vfh_resolution);
	//float delta_vel_dif=temp_v_dif_max/(vel_resolution-1);
	double dtheta=M_PI/180;
/*
	for(int i=0;i<vel_resolution;i++){
		temp_vr.push_back(temp_v-temp_v_dif_max/2+i*delta_vel_dif);
		temp_vl.push_back(temp_v+temp_v_dif_max/2-i*delta_vel_dif);
		temp_w.push_back( (temp_vr[i]-temp_vl[i])/(2*d) );
		if(temp_w[i]!=0){
			double temptemp_p=temp_v/temp_w[i];
			if(temptemp_p<0)
				temp_p.push_back(-temptemp_p);
			else
				temp_p.push_back(temptemp_p);
		}
		else{
			temp_p.push_back(0);
		}
*/

//		max_process_n.push_back((int)(M_PI/delta_theta[i]));
//	}
//set parameter of collision avoidance
	int n_circle_size=2*(int)( 2*(R+d_r)/grid_cell_size / 2 ) + ( (int)(2*(R+d_r)/grid_cell_size) / 2 ) % 2;
	jn_Rd=n_circle_size;
	in_Rd.reserve(n_circle_size);
	double temp_x;
	for(int j=0;j<jn_Rd;j++){
		temp_x=sqrt( (R+d_r)*(R+d_r) - ((j-jn_Rd/2)*grid_cell_size)*((j-jn_Rd/2)*grid_cell_size) );
		in_Rd.push_back(
			(int)( 2*(temp_x)/grid_cell_size / 2 ) + ( (int)(2*(temp_x)/grid_cell_size)  ) % 2 );
	}
//debug
/*
	for(int i=0;i<vel_resolution;i++){
		std::cout<<"vr,vl,w,p,dtheta,:"<<temp_vr[i]<<","<<temp_vl[i]<<","<<temp_w[i]<<","<<temp_p[i]<<","<<delta_theta[i]<<","<<max_process_n[i]<<"\n";
	}
	std::cout<<"R+d_r,int(R+d_r),Sc:"<<R+d_r<<","<<n_circle_size<<","<<grid_cell_size<<"\n";
	for(int j=0;j<jn_Rd;j++){
		std::cout<<"in_Rd["<<j<<"]:"<<in_Rd[j]<<"\n";
	}
*/
}

vfh_class::~vfh_class(){

}
void vfh_class::subscribe_cluster(void){
	queue.callOne(ros::WallDuration(1));
}
/*
void vfh_class::subscribe_pcl(void){
	queue.callOne(ros::WallDuration(1));
}
void vfh_class::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::fromROSMsg (*msg, pcl_data);
	ROS_INFO("into plc_callback");
}
*/
void vfh_class::cluster_callback(const obst_avoid::cluster_with_vel::ConstPtr& msg)
{
	cluster.clst=msg->clst;
	cluster.vel=msg->vel;
	EXECUTED_CALLBACK=true;
}
bool vfh_class::is_cluster(void){
	return EXECUTED_CALLBACK;
}
void vfh_class::set_grid_map(void){
	int grid_x,grid_z;

//renew
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution,grid_resolution), CV_8UC1);
	grid_map=m.clone();


	for(int i=0;i<cluster.clst.size();i++)
	{
		double vel_size=std::sqrt( std::pow(cluster.vel[i].x,2.0) + std::pow(cluster.vel[i].y,2.0) + std::pow(cluster.vel[i].z,2.0) );
		if( vel_size<1.1&&vel_size>0.1)
		{

		}
		for(int k=0;k<cluster.clst[i].pt.size();k++)
		{
			float x = (cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cluster.clst[i].pt[k].z/f;
			float y = (height/2 - cluster.clst[i].pt[k].y*ksize+ksize/2)*cluster.clst[i].pt[k].z/f;
			float z = cluster.clst[i].pt[k].z;
			if(z<grid_size/2){
				grid_z=(int)( (grid_size/2-z) /grid_cell_size);
				if(std::abs(x)<grid_size/2){
					grid_x=( (int)(2*(x+grid_size/2)/grid_cell_size)/2 ) + ( (int)(2*(x+grid_size/2)/grid_cell_size) ) % 2;
					grid_map.at<uint8_t/*uchar*/>(grid_z,grid_x)+=2;
					if(grid_map.at<uint8_t/*uchar*/>(grid_z,grid_x)>=255){
						grid_map.at<uint8_t/*uchar*/>(grid_z,grid_x)=255;
					}
				}
			}
		}
	}
	grid_map-=1;

}
void vfh_class::simulate_obstacle(void)
{
	float set_px=0.5;
	float set_py=1;
	float range=0.5;
	int nx,ny;
	int range_i=(int)(2*range/grid_cell_size/2) + (int)(2*range/grid_cell_size) % 2;
;
	transport_robot_to_gridn(set_px,set_py,nx,ny);
	for(int h=ny-range_i;h<ny+range;h++)
	{
		for(int w=nx-range_i;w<nx+range;w++)
		{
			grid_map.at<uint8_t>(h,w)=255;

		}
	}
}

void vfh_class::select_best_trajectory(void){
	double xr;
	double yr;
	int nx;
	int ny;
	for(int i=0;i<vfh_resolution;i++)
	{
		rank_trajectory[i]=0;
	}

/*
	for(int i=0;i<temp_p.size();i++){
		double theta=0;
		int j_max=15;//test param
		int j=0;
		bool LOOP_OUT=false;
		for(;j<j_max && (j<max_process_n[i] || i==temp_p.size()/2);j++,theta+=delta_theta[i]){
			if(temp_w[i]>0){
				xr=temp_p[i]*(cos(delta_theta[i]*j)-1);
				yr=temp_p[i]*sin(delta_theta[i]*j);
			}
			else if(temp_w[i]<0){
				xr=temp_p[i]*(1-cos(delta_theta[i]*j));
				yr=temp_p[i]*sin(delta_theta[i]*j);
			}
			else{
				xr=0;
				yr=j*R;
			}
			transport_robot_to_gridn(xr,yr,nx,ny);
			if( is_obstacle(nx,ny) ){
				LOOP_OUT=true;
				j--;
				rank_trajectory.push_back(j);
				break;
			}
		}
		if(!LOOP_OUT)
			rank_trajectory.push_back(j);
	}
*/
	float x0=0;
	float y0=0;
	float theta0=0;
	int max_search_n=20;
	for(int i=0;i<vfh_resolution;i++)
	{
		float x=x0;
		float y=y0;
		float theta=(min_angle+i*(max_angle-min_angle)/(vfh_resolution-1))*M_PI/180;
		float mv_length=R;
		for(int n=0;n<max_search_n;n++)
		{
			x=x0+mv_length*(-sin(theta))*n;
			y=y0+mv_length*(cos(theta))*n;
			transport_robot_to_gridn(x,y,nx,ny);

			if( is_obstacle(nx,ny)||n==max_search_n-1 ){
				//std::cout<<"i,n:"<<i<<","<<n<<"\n";
				rank_trajectory[i]=n;
				break;
			}
		}
	}

	float good_trajectory_value=100;
	good_trajectory_num=vfh_resolution/2;
	float evaluation_formula;
	float w_angle=0.5;
	float xp=0;//purpose
	float yp=5;
	float xc=0;//current
	float yc=0;
	float w_target=0.8;
	for(int i=0;i<vfh_resolution;i++){
		float theta=(min_angle+i*(max_angle-min_angle)/(vfh_resolution-1))*M_PI/180;
		//std::cout<<"i:"<<i<<"\n";
		//std::cout<<"max_angle-min_angle/vfh_resolution:"<<(max_angle-min_angle)/(vfh_resolution-1)<<"\n";
		//std::cout<<"i*(max_angle-min_angle)/(vfh_resolution-1):"<<i*(max_angle-min_angle)/(vfh_resolution-1)<<"\n";
		//std::cout<<"M_PI:"<<M_PI<<"\n";
		//std::cout<<"theta:"<<theta<<"\n";
		//std::cout<<"theta:"<<theta*180/M_PI<<"\n";
		float theta_half=vfh_resolution/2*std::abs((max_angle-min_angle)/(vfh_resolution-1))*M_PI/180;
		evaluation_formula=(max_search_n-rank_trajectory[i])
	+(std::abs(i-vfh_resolution/2)/(vfh_resolution/2))*max_search_n*w_angle
	+std::abs(std::atan(-(xp-xc)/(yp-yc))-theta)/theta_half*max_search_n*w_target;
		//std::cout<<"d_theta:"<<std::abs(std::atan(-(xp-xc)/(yp-yc))-theta)*180/M_PI<<"\n";
		//std::cout<<"rank_trajectory"<<i<<"]:"<<rank_trajectory[i]<<"\n";
		std::cout<<"trajectory["<<i<<"]:"<<evaluation_formula<<"\n";
		if(good_trajectory_value>evaluation_formula){
			good_trajectory_value=evaluation_formula;
			good_trajectory_num=i;
		}
	}

	std::cout<<"good tarjectory is "<<good_trajectory_num<<"\n";
	std::cout<<"good tarjectory value is "<<good_trajectory_value<<"\n";
	draw_best_trajectory(good_trajectory_num);
}
void vfh_class::transport_robot_to_gridn(const double& xr,const double& yr,int& n_xr,int& n_yr){
	double grid_x=xr+grid_size/2;
	double grid_y=grid_size/2-yr;

	transport_gridx_to_gridn(grid_x,grid_y,n_xr,n_yr);
}

void vfh_class::transport_gridx_to_gridn(const double& x,const double& y,int& n_x,int& n_y){
	n_x = (int)(2*x/grid_cell_size/2) + (int)(2*x/grid_cell_size) % 2;
	n_y = (int)(2*y/grid_cell_size/2) + (int)(2*y/grid_cell_size) % 2;
}

bool vfh_class::is_obstacle(const int nx,const int ny){

	for(int j=0;j<jn_Rd;j++){
		for(int i=-in_Rd[j]+nx;i<=in_Rd[j]+nx;i++){
			if((j-jn_Rd/2)+ny<0 || (j-jn_Rd/2)+ny>grid_resolution || i<0 || i>grid_resolution)
				return true;
			if(grid_map.at<uint8_t>((j-jn_Rd/2)+ny,i)>=	binary_threshold)
				return true;
		}
	}
	return false;
}
void vfh_class::draw_best_trajectory(const int& num){

	std::cout<<"draw_best_trajectory_start\n";

//	double good_p=temp_p[num];
//	double good_delta_theta=delta_theta[num];
//	double good_w=temp_w[num];
	int nx,ny;
	int nx_1=0;
	int ny_1=0;
	double x0=0;
	double y0=0;
	double theta0=0;
	float x=x0;
	float y=y0;
	std::cout<<"nun:"<<num<<"\n";
	//std::cout<<"rank_trajectory[num]:"<<rank_trajectory[num]<<"\n";

	transport_robot_to_gridn(x,y,nx_1,ny_1);

	for(int j=0;j<rank_trajectory[num] ;j++){
		float theta=(min_angle+num*(max_angle-min_angle)/(vfh_resolution-1))*M_PI/180;
		float mv_length=R;
		//std::cout<<"x:"<<x;
		//std::cout<<"y:"<<y;
		x=x0+mv_length*(-sin(theta))*j;
		y=x0+mv_length*(cos(theta))*j;
		transport_robot_to_gridn(x,y,nx,ny);
		if(j>0&&!(nx_1==nx&&ny_1==ny) ){
			cv::line(grid_map_view, cv::Point(nx_1, ny_1), cv::Point(nx, ny), cv::Scalar(0,255,255), 1, 4);
		}
		nx_1=nx;
		ny_1=ny;
	}
	std::cout<<"draw_best_trajectory\n";
}
void vfh_class::draw_all_trajectory(void){
	double xr;
	double yr;
	int nx;
	int ny;
	int nx_1;
	int ny_1;

	nx_1=0;
	ny_1=0;
	nx=0;
	ny=0;

	float x0=0;
	float y0=0;
	float theta0=0;
	int max_search_n=20;
	for(int i=0;i<vfh_resolution;i++)
	{

		float x=x0;
		float y=y0;
		float theta=(min_angle+i*(max_angle-min_angle)/(vfh_resolution-1))*M_PI/180;
		float mv_length=R;
		transport_robot_to_gridn(x0,y0,nx_1,ny_1);
		if(i==45){
			std::cout<<theta<<":("<<-sin(theta)<<","<<cos(theta)<<")\n";
		}

		for(int n=0;n<max_search_n;n++)
		{
			x=x0+mv_length*(-sin(theta))*n;
			y=y0+mv_length*(cos(theta))*n;
			transport_robot_to_gridn(x,y,nx,ny);

			if( is_obstacle(nx,ny)||n==max_search_n-1 ){
				break;
			}
			cv::line(grid_map_view, cv::Point(nx_1, ny_1), cv::Point(nx, ny), cv::Scalar(0,255,255), 1, 4);
			nx_1=nx;
			ny_1=ny;
		}
	}

}


void vfh_class::set_grid_map_view(void){
	for(int h=0;h<grid_resolution;h++){
		for(int w=0;w<grid_resolution;w++){
			grid_map_view.at<cv::Vec3b>(h,w)[1]=grid_map.at<uchar>(h,w);
			grid_map_view.at<cv::Vec3b>(h,w)[0]=0;
			grid_map_view.at<cv::Vec3b>(h,w)[2]=0;

		}
	}
}
void vfh_class::set_binary_grid_map_view(void){
	for(int h=0;h<grid_resolution;h++){
		for(int w=0;w<grid_resolution;w++){
			if(grid_map.at<uchar>(h,w)>=binary_threshold)
				binary_grid_map_view.at<cv::Vec3b>(h,w)[1]=255;
			else
				binary_grid_map_view.at<cv::Vec3b>(h,w)[1]=0;
		}
	}
}
void vfh_class::publish_grid_map_view(void){

	float robot_r=R;
	int robot_cell_size=(int)(robot_r/grid_cell_size);
//	int temp;
//	transport_gridx_to_gridn(robot_r,robot_r,robot_cell_size,temp);
	for(int h=grid_resolution/2-robot_cell_size;h<grid_resolution/2+robot_cell_size;h++){
		for(int w=grid_resolution/2-robot_cell_size;w<grid_resolution/2+robot_cell_size;w++){
			grid_map_view.at<cv::Vec3b>(h,w)[2]=255;
		}
	}

	grid_map_view.at<cv::Vec3b>(grid_resolution/2,grid_resolution/2)[0]=255;
	grid_map_view.at<cv::Vec3b>(grid_resolution/2,grid_resolution/2)[2]=255;
	grid_map_view.at<cv::Vec3b>(grid_resolution/2,grid_resolution/2)[1]=255;

	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=grid_map_view.clone();
	pub.publish(publish_cvimage->toImageMsg());
}
void vfh_class::publish_binary_grid_map_view(void){


	float robot_r=R;
	int robot_cell_size=(int)(robot_r/grid_cell_size);
//	int temp;
//	transport_gridx_to_gridn(robot_r,robot_r,robot_cell_size,temp);
/*
	for(int h=grid_resolution/2-robot_cell_size/2;h<grid_resolution/2+robot_cell_size/2;h++){
		for(int w=grid_resolution/2-robot_cell_size/2;w<grid_resolution/2+robot_cell_size/2;w++){
			binary_grid_map_view.at<cv::Vec3b>(h,w)[2]=255;
		}
	}
*/
	int cp_grid_map=grid_resolution/2;
	for(int j=0;j<jn_Rd;j++){
		for(int i=-in_Rd[j]+cp_grid_map;i<=in_Rd[j]+cp_grid_map;i++){
			binary_grid_map_view.at<cv::Vec3b>((j-jn_Rd/2)+grid_resolution/2,i)[2]=255;
		}
	}
	for(int h=cp_grid_map-robot_cell_size;h<cp_grid_map+robot_cell_size;h++){
		for(int w=grid_resolution/2-robot_cell_size;w<grid_resolution/2+robot_cell_size;w++){
			binary_grid_map_view.at<cv::Vec3b>(h,w)[0]=255;
			binary_grid_map_view.at<cv::Vec3b>(h,w)[2]=0;
		}
	}

	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=binary_grid_map_view.clone();
	pub2.publish(publish_cvimage->toImageMsg());
}

void vfh_class::publish_cloud(void)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB cloud_temp;

	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト

	for(int i=0;i<cluster.clst.size();i++)
	{
		for(int t=0;t<4;t++)
		{
			if(std::sqrt(std::pow(cluster.vel[i].x,2.0)+std::pow(cluster.vel[i].z,2.0))>1.1
			//||
			)
			{
				//continue;
			}
			for(int k=0;k<cluster.clst[i].pt.size();k++)
			{
				//cloud_temp.y=-(cluster.clst[i].pt[k].x*ksize-width/2)*cluster.clst[i].pt[k].z/f;
				//cloud_temp.z=((height/2-cluster.clst[i].pt[k].y*ksize)*cluster.clst[i].pt[k].z)/f+0.4125;
				cloud_temp.y=-(cluster.clst[i].pt[k].x*ksize+ksize/2-width/2)*cluster.clst[i].pt[k].z/f;
				cloud_temp.z=((height/2-cluster.clst[i].pt[k].y*ksize+ksize/2)*cluster.clst[i].pt[k].z)/f+0.4125;
				cloud_temp.x=cluster.clst[i].pt[k].z;
				cloud_temp.r=colors[i%12][0];
				cloud_temp.g=colors[i%12][1];
				cloud_temp.b=colors[i%12][2];
				//cloud_temp.x+=cluster.vel[i].z*t;
		    //cloud_temp.y+=-cluster.vel[i].x*t;
		    //cloud_temp.z+=cluster.vel[i].y*t;

				cloud->points.push_back(cloud_temp);
			}
		}
	}
	sensor_msgs::PointCloud2 edit_cloud;
	pcl::toROSMsg (*cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
	pc_pub.publish(edit_cloud);
}

void vfh_class::publish_velocity(void)
{
	int theta_i=good_trajectory_num;
	float theta=(min_angle+theta_i*(max_angle-min_angle)/(vfh_resolution-1))*M_PI/180;
	float dif;
	vel=0.2;
	float w=theta*(length/vel);
	dif=w*d;

	if(max_dif<std::abs(dif))
	{
		if(dif>0)
		{
			dif=max_dif;
		}
		else
		{
			dif=-max_dif;
		}
		//hold on
		if(rank_trajectory[theta_i]<z_th)
		{

		}
		else
		{

		}
	}
	wheelMsg.vel_l=(int)( (vel-dif)*1000);
	wheelMsg.vel_r=(int)( (vel+dif)*1000);
	pub_wheel.publish(wheelMsg);

}

void vfh_class::publish_velocity(float& vel,float& angvel)
{

	float dif;

	float w=angvel*(length/vel);
	dif=w*d;

	if(max_dif<std::abs(dif))
	{
		if(dif>0)
		{
			dif=max_dif;
		}
		else
		{
			dif=-max_dif;
		}
	}
	wheelMsg.vel_l=(int)( (vel-dif)*1000);
	wheelMsg.vel_r=(int)( (vel+dif)*1000);
	pub_wheel.publish(wheelMsg);

}
//-----
void vfh_class::set_param(const int& min_ang,const int& max_ang,const int& reso,
							const float& rob_r,const float& mrg_r,const float& mv_length,
							const float& max_vdif)
{
							min_angle=min_ang;
							max_angle=max_ang;
							vfh_resolution=reso;
							R=rob_r;
							d_r=mrg_r;
							length=mv_length;
							max_dif=max_vdif;
							not_select_angle.resize(max_angle-min_angle);
}

void vfh_class::set_grid_map(const std::vector<obst_avoid::point3d>& pt) {
	int grid_x, grid_z;


	//std::cout<<"set_grid_map\n";
	for (int k = 0; k<pt.size(); k++)
	{
		float x = (pt[k].x- width / 2)*pt[k].z / f;//(pt[k].x*ksize + ksize / 2 - width / 2)*pt[k].z / f;
		float y = (height / 2 - pt[k].y)*pt[k].z / f;//(height / 2 - pt[k].y*ksize + ksize / 2)*pt[k].z / f;
		float z = pt[k].z;
		//std::cout<<"pt["<<k<<"]:"<<pt[k]<<"\n";
		//std::cout<<"x,y,z:"<<x<<","<<y<<","<<z<<"\n";
		if (z < grid_size / 2) {
			grid_z = (int)((grid_size / 2 - z) / grid_cell_size);
			if (std::abs(x)<grid_size / 2) {
				grid_x = ((int)(2 * (x + grid_size / 2) / grid_cell_size) / 2) + ((int)(2 * (x + grid_size / 2) / grid_cell_size)) % 2;
				//grid_map.at<uint8_t/*uchar*/>(grid_z, grid_x) += 2;
				grid_map.at<uint8_t/*uchar*/>(grid_z, grid_x) = 255;
				if (grid_map.at<uint8_t/*uchar*/>(grid_z, grid_x) >= 255) {
					grid_map.at<uint8_t/*uchar*/>(grid_z, grid_x) = 255;
				}
			}
		}
	}
}

void  vfh_class::clear_grid_map(void)
{
	cv::Mat m = cv::Mat::zeros(cv::Size(grid_resolution, grid_resolution), CV_8UC1);
	grid_map = m.clone();
}

int vfh_class::select_best_trajectory(const cv::Point2f& x0,const float& theta0,const cv::Point2f& xp,
		const float w_target,const float w_angle) {

	double xr;
	double yr;
	int nx;
	int ny;
	for (int i = 0; i<vfh_resolution; i++)
	{
		rank_trajectory[i] = 0;
	}

	//float x0 = x0.x;
	//float y0 = x0.y;
	//float theta0 = 0;
	cv::Point2f xt0 = cv::Point2f(0,0);
	int max_search_n = 20;
	float mv_length = R;
	for (int i = 0; i<vfh_resolution; i++)
	{
		float x = xt0.x;
		float y = xt0.y;
		float theta = (min_angle + (float)i * (max_angle - min_angle) / ( vfh_resolution ) )*M_PI / 180;
		for (int n = 0; n < max_search_n; n++)
		{
			x = xt0.x + mv_length * (-sin(theta))*n;
			y = xt0.y + mv_length * (cos(theta))*n;
			transport_robot_to_gridn(x, y, nx, ny);

			if (is_obstacle(nx, ny) || n == max_search_n - 1) {
				//std::cout<<"i,n:"<<i<<","<<n<<"\n";
				rank_trajectory[i] = n;
				break;
			}
		}
	}

	float good_trajectory_value = 100;
	good_trajectory_num = vfh_resolution / 2;
	float evaluation_formula;
	//float w_angle = 0.5;
	//float xp = 0;//purpose
	//float yp = 5;
	float xc = x0.x;//current
	float yc = x0.y;
	//std::cout<<"x0,xp:"<<x0<<","<<xp<<"\n";
	//std::cout<<"std::atan(-(xp.x - xc) / (xp.y - yc)):"<<std::atan(-(xp.x - xc) / (xp.y - yc))<<"\n";

	//threshold length
	float th_ln=1.0;
	int th_ln_i=(int)(th_ln/mv_length)+1;
	// std::cout<<"th i,ln, mvlength,"<<th_ln_i<<","<<th_ln<<","<<mv_length<<"\n";
	for (int i = 0; i<vfh_resolution; i++) {
		if(not_select_angle[i])
		{
			//std::cout<<"i:"<<i<<"\n";
			continue;
		}

		float theta = (min_angle + (float)i * (max_angle - min_angle) / ( vfh_resolution ))*M_PI / 180;
		float theta_half = (float)vfh_resolution / 2 * std::abs((float)(max_angle - min_angle) / (vfh_resolution))*M_PI / 180;

		// std::cout<<"i,rank_trajectory:"<<i<<","<<rank_trajectory[i]<<"\n";
		if(th_ln_i>rank_trajectory[i])
		{
			// std::cout<<"i,rank_trajectory:"<<i<<","<<rank_trajectory[i]<<"\n";
			continue;
		}
		else{
			evaluation_formula = (max_search_n - rank_trajectory[i])
				+ (std::abs(i - vfh_resolution / 2) / (vfh_resolution / 2))*max_search_n*w_angle
				+ std::abs(std::atan(-(xp.x - xc) / (xp.y - yc))-theta0 - theta ) / theta_half * max_search_n*w_target;
		}
		//std::cout<<"aa:"<<theta_half<<"\n";
		//std::cout<<"evaluation_formula:"<<evaluation_formula<<"\n";
		if (good_trajectory_value > evaluation_formula)
		{
			good_trajectory_value = evaluation_formula;
			good_trajectory_num = i;
		}
	}
	//std::cout<<"good_trajectory_value:"<<good_trajectory_value<<"\n";
	if(good_trajectory_value==100){
		return -1;
	}

	return good_trajectory_num;
	//return ((min_angle + good_trajectory_num * (max_angle - min_angle) / (vfh_resolution))*M_PI / 180);
}

void vfh_class::set_not_select_angle(std::vector<bool>& not_select_angle_temp)
{
	std::cout<<"not_select_angle.size():"<<not_select_angle.size()<<"\n";
	for(int i=0;i<not_select_angle.size();i++)
	{
		not_select_angle[i]=not_select_angle_temp[i];

		if(not_select_angle[i])
		{
			//std::cout<<"i:"<<i<<"\n";
		}

	}
}
bool vfh_class::draw_line(float x0,float y0,float x1,float y1)
{
	int nx0,ny0,nx1,ny1;
	transport_gridx_to_gridn(x0,y0,nx0,ny0);
	transport_gridx_to_gridn(x1,y1,nx1,ny1);
	if(nx0>vfh_resolution||nx0<0 || ny0>vfh_resolution||ny0<0){
		std::cout<<"x0>vfh_resolution||x0<0 || y0>vfh_resolution||y0<0 is true. \n";
		return false;
	}
	if(nx1>vfh_resolution||nx1<0 || ny1>vfh_resolution||ny1<0){
		std::cout<<"x1>vfh_resolution||x1<0 || y1>vfh_resolution||y1<0 is true. \n";
		return false;
	}
	cv::line(grid_map_view, cv::Point(nx0, ny0), cv::Point(nx1, ny1), cv::Scalar(0,255,255), 1, 4);
	return true;
}
void vfh_class::draw_circle(const float& x0,const float& y0)
{
	int nx0,ny0;
	transport_gridx_to_gridn(x0,y0,nx0,ny0);
	cv::circle(grid_map_view,cv::Point(nx0, ny0),3,cv::Scalar(0,0,200), -1, CV_AA);
}
/*
int main(int argc,char **argv){
	ros::init(argc,argv,"vfh_class_test");
	vfh_class vfh;
	std::cout<<"ready\n";

	while(ros::ok()){
		vfh.subscribe_cluster();
		std::cout<<"subscribe_cluster\n";
		if(vfh.is_cluster()){
		std::cout<<"is_cluster\n";
			vfh.set_grid_map();
		std::cout<<"set_grid_map\n";
	//	vfh.simulate_obstacle();
		std::cout<<"simulate_obstacle\n";
			vfh.set_grid_map_view();
		std::cout<<"set_grid_map_view\n";
			vfh.select_best_trajectory();
		std::cout<<"select_best_trajectory\n";
		//	vfh.draw_all_trajectory();
			vfh.publish_grid_map_view();
		std::cout<<"publish_grid_map_view\n";
			vfh.set_binary_grid_map_view();
		std::cout<<"set_binary_grid_map_view\n";
			vfh.publish_binary_grid_map_view();
		std::cout<<"publish_binary_grid_map_view\n";
		vfh.publish_cloud();
		std::cout<<"publish_cloud\n";
		vfh.publish_velocity();
		}
	}

	return 0;
}
*/
