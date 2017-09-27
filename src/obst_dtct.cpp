#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>

#include<fstream>

image_transport::Subscriber sub_depth;
image_transport::Publisher comp_nan;
cv_bridge::CvImagePtr depthimg;
int width=672;
int height=376;
int n=0;
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)	
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    depthimg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//BGR8);
    cv::Mat depth_img;	
    cv::Mat edit_img = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);	
    depth_img=depthimg->image.clone();
    float depth;
    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++){
	    depth=depth_img.at<float>(j,i);
	    if(!std::isnan(depth)&&depth>0)
	        edit_img.at<float>(j,i)=(int)(depth*10)/10.0;//10cmごとのdataに変換
	    }
	}
	cv::Mat view_img = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);	
			
	cv::Vec3b color;
	float max_distance=5.0;
	for(int i=0;i<width;i++){
	    for(int j=0;j<height;j++){
		if(edit_img.at<float>(j,i)==0){
		    color.val[0]=0;
		    color.val[1]=0;
		    color.val[2]=255;
		}
		else{//if(edit_img.at<float>(j,i)<=max_distance){
		    int color_temp=(int)(edit_img.at<float>(j,i)/max_distance*255);
		    if(color_temp>255)
		        color_temp=255;
		    color.val[0]=color_temp;
		    color.val[1]=color_temp;
		    color.val[2]=color_temp;
		}	
		view_img.at<cv::Vec3b>(j,i)=color;
	    }
	}
	comp_nan=it.advertise("comp_nan",1);
	cv_bridge::CvImagePtr PubDepth(new cv_bridge::CvImage);
	PubDepth->encoding=sensor_msgs::image_encodings::BGR8;
	PubDepth->image=view_img.clone();
	comp_nan.publish(PubDepth->toImageMsg());
	if(n++==0){
		//ofstream Data converted into each 10cm
		std::ofstream ofs("depth_graph.csv",std::ios::app);
/* 	    ofs << ",";
		for(int j=0;j<width;j++)
		    ofs << j << ",";
		ofs << std::endl;
		for(int i=0;i<height;i++){
		    ofs << i << ",";
		    for(int j=0;j<width;j++)
		        ofs << edit_img.at<float>(i,j) << ",";
		    ofs << std::endl;
*/
		for(int i=0;i<height;i++){
			for(int j=0;j<width;j++)
				ofs<<j<<" "<<i<<" "<<edit_img.at<float>(i,j)<<std::endl;
   		}
	}

	//
		
//		cv::Mat edit_img = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);	
	for(int i=0;i<height;i++){
	   	for(int j=0;j<width;j++){
				
				
				
		}
	}
	
}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"obstacle_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	sub_depth=it.subscribe("/zed/depth/depth_registered",1,
		&depthImageCallback);
	ros::spin();
	return 0;
}
