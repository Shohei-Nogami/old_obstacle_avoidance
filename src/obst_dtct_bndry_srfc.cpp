#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>


image_transport::Subscriber sub_depth;
image_transport::Publisher pub_edge;
cv_bridge::CvImagePtr depthimg;
int width=672;
int height=376;

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)	
{
//    ros::NodeHandle nh;
//    image_transport::ImageTransport it(nh);
    
    depthimg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//BGR8);
    cv::Mat depth_img;	
    cv::Mat edit_img = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);	
    depth_img=depthimg->image.clone();
    float depth;
    float max_distance=5.0;
    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++){
	    depth=depth_img.at<float>(j,i);
	    if(!std::isnan(depth)&&!std::isinf(depth))
	        edit_img.at<float>(j,i)=(int)(depth*10)/10.0;//10cmごとのdataに変換
	    if(std::isinf(depth))
		edit_img.at<float>(j,i) = max_distance;
        }
    }
	cv::Mat view_img1 = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);	
    cv::Mat view_img2 = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);	
	cv::Mat view_img3 = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);	

	cv::Vec3b color;
	cv::Vec3b white_color;
	white_color.val[0]=255;
	white_color.val[1]=255;
	white_color.val[2]=255;
//境界面を白く塗りつぶす
//縦方向
	float y;
	float f=350.505;
    int cx=(int)354.676;
    int cy=(int)191.479;
	float height_cam=0.215;
	float height_all=0.7;
	float threshold=0.1;
	for(int i=0;i<width;i++){
	    for(int j=0;j<height-1;j++){
		  depth=edit_img.at<float>(j,i);
		  y=-(j-cy)*depth/f+height_cam;
//			std::cout<<"(Y,z,y):("<<j<<","<<depth<<","<<y<<")\n";
		  if(y>=height_all||y<=0.05){
	      	depth=edit_img.at<float>(j+1,i);
			continue ;
		  }
	      if(depth!=0&&std::abs(depth-edit_img.at<float>(j+1,i))<=threshold)	
	        view_img1.at<cv::Vec3b>(j,i)=white_color;
	      depth=edit_img.at<float>(j+1,i);
	    }
	}
	//横方向
	for(int j=0;j<height;j++){
		for(int i=0;i<width-1;i++){
	  	  depth=edit_img.at<float>(j,i);
		  y=-(j-cy)*depth/f+height_cam;
		  if(y>=height_all||y<=0){
	      	depth=edit_img.at<float>(j,i+1);
			continue ;
		  }
	      if(depth!=0&&std::abs(depth-edit_img.at<float>(j,i+1))<=threshold)
	        view_img1.at<cv::Vec3b>(j,i)=white_color;
	      depth=edit_img.at<float>(j,i+1);
	    }
	}

	cv_bridge::CvImagePtr PubDepth(new cv_bridge::CvImage);
	PubDepth->encoding=sensor_msgs::image_encodings::BGR8;
	PubDepth->image=view_img1.clone();	
	pub_edge.publish(PubDepth->toImageMsg());


}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"obstacle_detection_boundry_surface");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	sub_depth=it.subscribe("/zed/depth/depth_registered",1,
		&depthImageCallback);
	pub_edge=it.advertise("output_Limage",1);
	ros::spin();
	return 0;
}
