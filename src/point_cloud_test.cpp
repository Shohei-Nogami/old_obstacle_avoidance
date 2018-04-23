#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/visualization/cloud_viewer.h>

#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>


class AnalysisPointCloud
{
private:
	ros::NodeHandle apc;
	ros::NodeHandle apc2;
	ros::NodeHandle apcp;
	ros::Subscriber pc_sub;
	ros::Subscriber pc_sub2;

	ros::Publisher pc_pub;
	ros::Publisher pc_pub2;

  ros::NodeHandle nh_im_pub;
  image_transport::Publisher im_pub;
  image_transport::ImageTransport it_pub;

//	uint32_t height;
//	uint32_t width;
	std::vector<sensor_msgs::PointField> fields;
	bool is_bigendian;
	uint32_t  point_step;
	uint32_t  row_step;
	std::vector<uint8_t> data;
	bool is_dense;
	std::string name;
	uint32_t offset;
	uint8_t  datatype;
	uint32_t count;
  cv::Mat transed_mat_temp;
  cv_bridge::CvImagePtr cvbridge_image;
  const int width=672;
  const int height=376;
public:
	ros::CallbackQueue pc_queue;
	ros::CallbackQueue pc_queue2;
	AnalysisPointCloud()
    :it_pub(nh_im_pub)
	{
		apc.setCallbackQueue(&pc_queue);
//		pc_sub = apc.subscribe("/camera/depth_registered/points",1,&AnalysisPointCloud::pointcloud_callback,this);
		apc2.setCallbackQueue(&pc_queue2);
		pc_sub2 = apc2.subscribe("/zed/point_cloud/cloud_registered",1,&AnalysisPointCloud::processing_pc,this);
		pc_pub = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud", 1);
		pc_pub2 = apcp.advertise<sensor_msgs::PointCloud2>("edit_cloud2", 1);
    im_pub=it_pub.advertise("transed_image",1);

    cv::Mat m = cv::Mat::zeros(cv::Size(width,height), CV_8UC3);
    transed_mat_temp=m.clone();
	};
	~AnalysisPointCloud(){};
	//void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
	void processing_pc(const sensor_msgs::PointCloud2::ConstPtr& ppc_msg);
	cv::Mat& get_transed_image(void);
	void publish_image(cv::Mat& req_image);
};


void AnalysisPointCloud::processing_pc(const sensor_msgs::PointCloud2::ConstPtr& ppc_msg)
{
	std::cout << "1" << std::endl;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA> test_cloud;
  std::cout << "2" << std::endl;
	pcl::fromROSMsg (*ppc_msg, test_cloud);
	std::cout << "get_point_cloud" << std::endl;

/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
	pcl::VoxelGrid<pcl::PointXYZ> vgf;
	vgf.setInputCloud (test_cloud);
	vgf.setLeafSize (0.1f, 0.1f, 0.1f);
	vgf.filter (*filtered_cloud);
*/
  std::cout << "test_cloud->points.size(),h,ps/h:" << test_cloud.points.size()<<","<<height<<","<<test_cloud.points.size()/height<<","<<'\n';
	for(int i=0;i<test_cloud.points.size();i++)
	{
		// test_cloud->points[i].x+=1.0;
		//		transed_mat_temp.at<cv::Vec3b>(0,0)[0]=test_cloud.points[i].b;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[0]= test_cloud.points[i].b;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[1]=test_cloud.points[i].g;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[2]=test_cloud.points[i].r;
    // transed_mat_temp.at<uint8_t>(i/width,i%width)[0]=test_cloud.points[i].b;
    // transed_mat_temp.at<uint8_t>(i/width,i%width)[1]=test_cloud.points[i].g;
    // transed_mat_temp.at<uint8_t>(i/width,i%width)[2]=test_cloud.points[i].r;
		/*if(std::isnan(test_cloud.points[i].z)||std::isinf(test_cloud.points[i].z)){
			std::cout<<"(z,r,g,b,a):("<<+test_cloud.points[i].z
				<<","<<+test_cloud.points[i].r
				<<","<<+test_cloud.points[i].g
				<<","<<+test_cloud.points[i].b
				<<","<<+test_cloud.points[i].a
				<<"\n";
		}
		*/
		//std::cout << test_cloud.points[i].x << ", " << test_cloud.points[i].y << ", " << test_cloud.points[i].z << std::endl;
	//", " << +test_cloud.points[i].r << ", " << +test_cloud.points[i].g << ", " << +test_cloud.points[i].b << std::endl;
		if(!ros::ok())
			break;
	}
	std::cout << "edit_cloud" << std::endl;
	sensor_msgs::PointCloud2 edit_cloud, edit_cloud2;
	pcl::toROSMsg (test_cloud, edit_cloud);
	// pcl::toROSMsg (*filtered_cloud, edit_cloud2);

	pc_pub.publish(edit_cloud);
	// pc_pub2.publish(edit_cloud2);
	std::cout << "publish_cloud" << std::endl;

}
cv::Mat& AnalysisPointCloud::get_transed_image(void){
	return transed_mat_temp;
}

void AnalysisPointCloud::publish_image(cv::Mat& req_image){
  cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	publish_cvimage->image=req_image.clone();
	im_pub.publish(publish_cvimage->toImageMsg());
}

//使用しない関数のcallback_func
/*
void AnalysisPointCloud::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
	height = pc_msg -> height;
	width = pc_msg -> width;
	fields = pc_msg -> fields;
	is_bigendian = pc_msg -> is_bigendian;
	point_step = pc_msg -> point_step;
	row_step = pc_msg -> row_step;
	data = pc_msg -> data;
	is_dense = pc_msg -> is_dense;

	std::cout << "height: " << height << "\n" << "width: " << width << "\n" << "is_bigendian: " << is_bigendian <<  "\n" << "point_step: " << point_step << "\n" << "row_step: " << row_step << "\n" << "data_size: " << data.size() << "\n" << "is_dense: " << is_dense << std::endl;


	for(int i=0;i<fields.size();i++)
	{
		name = fields[i].name;
		offset = fields[i].offset;
		datatype = fields[i].datatype;
		count = fields[i].count;
		//std::cout << "name: " << name << ", " << "offset: " << offset << ", " << "datatype: " << +datatype <<  ", " << "count: " << count << std::endl;
		//std::cout << "name: " << name << std::endl;
		//std::cout << "offset: " << offset << std::endl;
		//std::cout << "datatype: " << datatype << std::endl;
		//std::cout << "count: " << count << std::endl;
	}


	for(int i=0;i<100;i++)
	{
		std::cout << "data[" << i << "]: " << +data[i] << std::endl;
		if(!ros::ok())
			break;
	}
}
*/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_test");
	AnalysisPointCloud apc;
	while(ros::ok()){
		//apc.pc_queue.callOne(ros::WallDuration(1));//使用しない関数
		std::cout << "0" << std::endl;
		apc.pc_queue2.callOne(ros::WallDuration(1));
		apc.publish_image(apc.get_transed_image());
	}
	return 0;
}
