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

#include"time_class.h"

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

//  uint32_t height;
//  uint32_t width;
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
  pcl::PointCloud<pcl::PointXYZRGBA> test_cloud;
  pcl::PointCloud<pcl::PointXYZ> test_cloud2;
  cv::Mat transed_mat_temp;
  cv_bridge::CvImagePtr cvbridge_image;
  const int width=672;
  const int height=376;
  const double map_size_z=16.04; //[m]
  const double map_size_x=8.04;  //[m]
  static const int map_size_nz=401;  //map height [pixcel]
  static const int map_size_nx=201;  //map width  [pixcel]
  const double cell_size=0.04;//[cm]
public:
  ros::CallbackQueue pc_queue;
  ros::CallbackQueue pc_queue2;
  AnalysisPointCloud()
    :it_pub(nh_im_pub)
  {
    apc.setCallbackQueue(&pc_queue);
//    pc_sub = apc.subscribe("/camera/depth_registered/points",1,&AnalysisPointCloud::pointcloud_callback,this);
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
  void convert_cvmat(void);
  void convert_dem(void);
  bool convert_coordinate_xz_nxz(const float x,const float z,int& nx,int& nz);
  void invert_coordinate_xz_nxz(const int& nx,const int& nz,float& x,float& z);
};


void AnalysisPointCloud::processing_pc(const sensor_msgs::PointCloud2::ConstPtr& ppc_msg)
{
//  std::cout << "1" << std::endl;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud;
//  std::cout << "2" << std::endl;
//  pcl::fromROSMsg (*ppc_msg, test_cloud);
	std::cout<<ppc_msg->header<<"\n";
  pcl::fromROSMsg(*ppc_msg, test_cloud2);
//  std::cout << "get_point_cloud" << std::endl;
  /*
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> vgf;
    vgf.setInputCloud (test_cloud);
    vgf.setLeafSize (0.1f, 0.1f, 0.1f);
    vgf.filter (*filtered_cloud);
  
  std::cout << "test_cloud->points.size(),h,ps/h:" << test_cloud.points.size()<<","<<height<<","<<test_cloud.points.size()/height<<","<<'\n';
  for(int i=0;i<test_cloud.points.size();i++)
  {
    // test_cloud->points[i].x+=1.0;
    //    transed_mat_temp.at<cv::Vec3b>(0,0)[0]=test_cloud.points[i].b;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[0]= test_cloud.points[i].b;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[1]=test_cloud.points[i].g;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[2]=test_cloud.points[i].r;
    // transed_mat_temp.at<uint8_t>(i/width,i%width)[0]=test_cloud.points[i].b;
    // transed_mat_temp.at<uint8_t>(i/width,i%width)[1]=test_cloud.points[i].g;
    // transed_mat_temp.at<uint8_t>(i/width,i%width)[2]=test_cloud.points[i].r;
    if(std::isnan(test_cloud.points[i].z)||std::isinf(test_cloud.points[i].z)){
      std::cout<<"(z,r,g,b,a):("<<+test_cloud.points[i].z
        <<","<<+test_cloud.points[i].r
        <<","<<+test_cloud.points[i].g
        <<","<<+test_cloud.points[i].b
        <<","<<+test_cloud.points[i].a
        <<"\n";
    }
    
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

  */
}

void AnalysisPointCloud::convert_cvmat(void){
  std::cout << "test_cloud->points.size(),h,ps/h:" << test_cloud.points.size()<<","<<height<<","<<test_cloud.points.size()/height<<","<<'\n';
  for(int i=0;i<test_cloud.points.size();i++)
  {
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[0]= test_cloud.points[i].b;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[1]=test_cloud.points[i].g;
    transed_mat_temp.at<cv::Vec3b>(i/width,i%width)[2]=test_cloud.points[i].r;
    if(!ros::ok())
      break;
  }

}
void AnalysisPointCloud::convert_dem(void){
/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> vgf;
  vgf.setInputCloud (test_cloud);
  vgf.setLeafSize (0.1f, 0.1f, 0.1f);
  vgf.filter (*filtered_cloud);
*/
  int nx,nz;
//  std::cout << "test_cloud->points.size(),h,ps/h:" << test_cloud.points.size()<<","<<height<<","<<test_cloud.points.size()/height<<","<<'\n';
  for(int i=0;i<test_cloud2.points.size();i++)
  {
    if(convert_coordinate_xz_nxz(test_cloud2.points[i].y,test_cloud2.points[i].x,nx,nz)){
      invert_coordinate_xz_nxz(nx,nz,test_cloud2.points[i].y,test_cloud2.points[i].x);
      
    }
    test_cloud2.points[i].x+=3.0;
    if(!ros::ok())
      break;
  }
//  std::cout << "edit_cloud" << std::endl;
  sensor_msgs::PointCloud2 edit_cloud, edit_cloud2;
  pcl::toROSMsg (test_cloud2, edit_cloud);
  // pcl::toROSMsg (*filtered_cloud, edit_cloud2);
	std::cout<<edit_cloud.header<<"\n";
  pc_pub.publish(edit_cloud);
  // pc_pub2.publish(edit_cloud2);
//  std::cout << "publish_cloud" << std::endl;

}
bool AnalysisPointCloud::convert_coordinate_xz_nxz(const float x,const float z,int& nx,int& nz){
  nx = (int)(2*x/cell_size/2) + (int)(2*x/cell_size) % 2;
  nz = (int)(2*z/cell_size/2) + (int)(2*z/cell_size) % 2;
  if(map_size_nx<std::abs(nx)||map_size_nz<nz){
    return false;
  }
  else{
    nx+=map_size_nx/2;
    nz=map_size_nz-nz;
    return true;
  }
}
void AnalysisPointCloud::invert_coordinate_xz_nxz(const int& nx,const int& nz,float& x,float& z){
  x=(nx-map_size_nx/2)*cell_size;
  z=(map_size_nz-nz)*cell_size;

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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_test");
  AnalysisPointCloud apc;
  time_class time_cls;
  while(ros::ok()){
    //apc.pc_queue.callOne(ros::WallDuration(1));//使用しない関数
//    std::cout << "0" << std::endl;
    apc.pc_queue2.callOne(ros::WallDuration(1));
    apc.convert_dem();
//    apc.publish_image(apc.get_transed_image());
    time_cls.set_time();
    std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
  }
  return 0;
}
