#include"detect_objects_class.h"



detect_objects::detect_objects()
	:it_pub1(nh_pub1),it_pub2(nh_pub2),EXECUTED_CALLBACK(false),cloud(new pcl::PointCloud<pcl::PointXYZ>),inliers (new pcl::PointIndices),coefficients(new pcl::ModelCoefficients)//,index_img(new std::vector<index_image>[map_size_nz][map_size_nx])//,Eclusted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{

	//publisher and subscriber
	pub1=it_pub1.advertise("detected_objects_image",1);//test string
	pub2=it_pub2.advertise("depth_image_for_synchro",1);//test string

	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("depth_by_optflw_node",1,&detect_objects::image_callback,this);
	//sub=nh_sub.subscribe("/zed/depth/depth_registered",1,&detect_objects::image_callback,this);


	pub_empty = nh_pub3.advertise<std_msgs::Empty>("response", 1);
  pub_cluster = nh_pub4.advertise<obst_avoid::cluster>("cluster", 1);
	pc_pub1 = nh_pubpcl1.advertise<sensor_msgs::PointCloud2>("filted_pcl", 1);
	pc_pub2 = nh_pubpcl2.advertise<sensor_msgs::PointCloud2>("clusted_pcl", 1);



//  cloud->width  = width;
//  cloud->height = height;
//  cloud->points.resize (cloud->width * cloud->height);

//床面抽出
	seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出

	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (500);//RANSACの繰り返し回数
	seg.setDistanceThreshold (0.10);//モデルとどのくらい離れていてもいいか???謎
	seg.setAxis(Eigen::Vector3f (0.0,0.0,1.0));//法線ベクトル
	seg.setEpsAngle(15.0f * (M_PI/180.0f));//許容出来る平面

	filted_image=cv::Mat::zeros(cv::Size(width,height), CV_32FC1);

	map_wf=10;
	map_hf=10;
	reso=0.1;
	cx=0;
	cy=0;

	img_3d=cv::Mat::zeros(cv::Size(width,height), CV_32FC3);
	index_to_gm=cv::Mat::zeros(cv::Size(width,height), CV_32SC2);
	grid_map=cv::Mat::zeros(cv::Size((int)(map_wf/reso),(int)(map_hf/reso)), CV_32SC1);
	cluster_num=cv::Mat::zeros(cv::Size((int)(map_wf/reso),(int)(map_hf/reso)), CV_32SC1);
	cluster_size=0;
	cluster_count.reserve((int)(map_wf/reso)*(int)(map_hf/reso));
}
detect_objects::~detect_objects(){
}

void detect_objects::subscribe_depth_image(void){
	queue.callOne(ros::WallDuration(1));
}
void detect_objects::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		cvbridge_image=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
		EXECUTED_CALLBACK=true;
	}
  catch(cv_bridge::Exception& e) {
	std::cout<<"depth_image_callback Error \n";
	  ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
	  msg->encoding.c_str());
	  return ;
  }
}
bool detect_objects::is_cvbridge_image(void){

	return EXECUTED_CALLBACK;
}
void detect_objects::set_depth_image(void){
	depth_image=cvbridge_image->image.clone();
}

void detect_objects::ground_estimation_from_image(const float& y_th,const float& cam_y,float& a,float& b,float& c,float& d){
  	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
  float x_temp;
  float y_temp;
  float z_temp;
  pcl::PointXYZ p_temp;
  //ground_points->points.clear();
  //std::cout<<"depth_image.empty():"<<depth_image.empty()<<"\n";
  ground_points->points.reserve(height/2*width);
  for(int h=height/2+1;h<height;h++){
    for(int w=0;w<width;w++){
      z_temp=depth_image.at<float>(h,w);
      if(z_temp>0.5&&!std::isinf(z_temp)){
        y_temp=(height/2-h)*z_temp/f;
	//std::cout<<"y_temp:"<<y_temp<<"\n";

        if(std::abs(y_temp+cam_y)<y_th){
          x_temp=-( ((float)w-(float)width/2)*z_temp/f-cam_y );

          p_temp.x=z_temp;
          p_temp.y=x_temp;
          p_temp.z=y_temp;
          ground_points->points.push_back(p_temp);
        }
      }
    }
  }
  ground_points->width=ground_points->points.size();
  ground_points->height=1;
	std::cout<<"ground_points->points.size():"<<ground_points->points.size()<<"\n";
  seg.setInputCloud (ground_points);

	seg.segment (*inliers, *coefficients);
	std::cout << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;
   a=coefficients->values[0];
   b=coefficients->values[1];
	c=coefficients->values[2];
	d=coefficients->values[3];

}

void detect_objects::publish_response(void)
{
	pub_empty.publish(emptymsg);
}
int main(int argc,char **argv){
	ros::init(argc,argv,"detect_objects_class_test");
	detect_objects dtct_obj;
  time_class time_cls;
	image_class img_cls;
	odometry_class odm_cls;
	img_cls.define_variable();
	std::cout<<"defined class\n";
	float x,z;
	int nx,nz;
	bool tf;
	bool WITH_GRIDMAP=false;
	while(ros::ok()){

		std::cout<<"process_start:"<<time_cls.get_time_now()<<"\n";

    img_cls.set_image();

		std::cout<<"2:set_image:"<<time_cls.get_time_now()<<"\n";


		dtct_obj.subscribe_depth_image();

		std::cout<<"2:subscribe_depth_image:"<<time_cls.get_time_now()<<"\n";

		time_cls.set_time();

		std::cout<<"2:set_time:"<<time_cls.get_time_now()<<"\n";

		odm_cls.subscribe_msgs();

		std::cout<<"2:subscribe_msgs:"<<time_cls.get_time_now()<<"\n";

		odm_cls.set_data();

		std::cout<<"2:set_data:"<<time_cls.get_time_now()<<"\n";

		odm_cls.set_velocity();

		std::cout<<"2:set_velocity:"<<time_cls.get_time_now()<<"\n";

		if(!dtct_obj.is_cvbridge_image()||!img_cls.is_cur_image())
			continue;
/*
		std::cout<<"3:subscribe_depth_image:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.subscribe_opticalflow();

		std::cout<<"3:subscribe_depth_image:"<<time_cls.get_time_now()<<"\n";

		dtct_obj.subsuctibe_matching();

		std::cout<<"3.5:subscribe_opticalflow:"<<time_cls.get_time_now()<<"\n";
*/

		dtct_obj.set_depth_image();

		std::cout<<"4:set_depth_image:"<<time_cls.get_time_now()<<"\n";
		if(!WITH_GRIDMAP){
			dtct_obj.filter_process();//0.09

			std::cout<<"9:filter_process:"<<time_cls.get_time_now()<<"\n";

			dtct_obj.density_based_clustering(img_cls.get_cur_image_by_ref());//0.27

			std::cout<<"9:clusterig_by_density_based:"<<time_cls.get_time_now()<<"\n";
		}
		else{
			dtct_obj.conv_depth_image();
			std::cout<<"9:conv_depth_image:"<<time_cls.get_time_now()<<"\n";

			dtct_obj.create_grid_map();
			std::cout<<"9:create_grid_map:"<<time_cls.get_time_now()<<"\n";
			dtct_obj.dbscan_with_gm();
			std::cout<<"9:dbscan_with_gm:"<<time_cls.get_time_now()<<"\n";
			dtct_obj.set_cluster();
			std::cout<<"9:set_cluster:"<<time_cls.get_time_now()<<"\n";
		}
		//img_cls.publish_debug_image( dtct_obj.draw_cluster(img_cls.get_cur_image_by_ref() ) );
		dtct_obj.draw_cluster();


		std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
		dtct_obj.publish_cluster(odm_cls.get_velocity(),odm_cls.get_angular_velocity(),time_cls.get_delta_time());
		dtct_obj.publish_response();
	}

	return 0;
}
