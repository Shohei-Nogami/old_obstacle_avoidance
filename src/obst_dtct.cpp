#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
//#include<fstream>

image_transport::Subscriber sub_depth;
image_transport::Publisher comp_nan;
image_transport::Publisher dtct_area;
cv_bridge::CvImagePtr depthimg;
int width=672;
int height=376;

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

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
      	view_img1.at<cv::Vec3b>(j,i)=color;
      }
  }
  //wheter depth value is nan
  comp_nan=it.advertise("comp_nan",1);
  cv_bridge::CvImagePtr PubDepth(new cv_bridge::CvImage);
  PubDepth->encoding=sensor_msgs::image_encodings::BGR8;
  PubDepth->image=view_img1.clone();
  comp_nan.publish(PubDepth->toImageMsg());
 
  float depth1=0,depth2=0;
  float fu=350.505;
  //上から下に
  if(edit_img.at<float>(0,0)!=0)
    depth2=edit_img.at<float>(0,0);
  int cy=(int)191.479;
  float height_cam=0.215;
  float height_all=0.7;
  float y;

  for(int j=0;j<height;j++){
    for(int i=0;i<width-1;i++){
      y=-(j-cy)*depth/fu+height_cam;
	  if(y>=height_all||y<=0.10)
		continue;
      if(edit_img.at<float>(j,i+1)!=0)
        depth2=edit_img.at<float>(j,i+1);
//	std::cout<<"j,j+1"<<edit_img.at<float>(j,i)<<","<<edit_img.at<float>(j+1,i)<<std::endl;
      if(depth2!=0&&edit_img.at<float>(j,i)==edit_img.at<float>(j,i+1)
		&&edit_img.at<float>(j,i)!=0){
//	std::cout<<"j,j+1"<<edit_img.at<float>(j,i)<<","<<edit_img.at<float>(j+1,i)<<std::endl;
        depth1=depth2;
        view_img2.at<cv::Vec3b>(j,i)=white_color;
      }
	
  /*    if(edit_img.at<float>(j,i)==0
        &&edit_img.at<float>(j,i+1)==depth1){
          //calc number of nan
//	std::cout<<"j,j+1"<<edit_img.at<float>(j,i)<<","<<edit_img.at<float>(j+1,i)<<std::endl;
          int count_nan=0;
    	    for(int k=i;edit_img.at<float>(j,k)==0;k--)
           		 count_nan++;
    	    double space= count_nan*depth/fu;//nanの距離
    	    if(space<=0.10){//<=30cm
	       	for(int k=i;edit_img.at<float>(j,k)==0;k--)
                	view_img2.at<cv::Vec3b>(j,k)=white_color;
          }//if(space<=thresold)

      }//if  |nan|nan|depth|
*/      //各行ごとを白く塗りつぶす 完了
    }
  }

/*
  if(!std::isnan(edit_img.at<float>(0,0)))
    depth2=edit_img.at<float>(0,0);
  for(int j=0;j<width;j++){
    for(int i=0;i<height-1;i++){
      if(!std::isnan(edit_img.at<float>(j,i+1)))
        depth2=edit_img.at<float>(j,i+1);
      if(!std::isnan(depth2)&&edit_img.at<float>(j,i)==edit_img.at<float>(j,i+1)){
        depth1=depth2;
        view_img2.at<cv::Vec3b>(j,i)=white_color;
      }
      if(edit_img.at<float>(j,i)==0&&edit_img.at<float>(j,i+1)==depth1){
          //calc number of nan
          int count_nan=0;
    	    for(int k=i;std::isnan(edit_img.at<float>(j,k));k--)
            count_nan++;
    	    double space= count_nan*depth/fu;//nanの距離
    	    if(space<=0.30){//<=30cm
        		for(int k=i;std::isnan(edit_img.at<float>(j,k));k--)
              view_img2.at<cv::Vec3b>(j,i)=white_color;
          }//if(space<=thresold)
      }//if  |nan|nan|depth|
      //各行ごとを白く塗りつぶす 完了
    }
  }
*/
//  cv_bridge::CvImagePtr PubDepth(new cv_bridge::CvImage);
//  PubDepth->encoding=sensor_msgs::image_encodings::BGR8;
  dtct_area=it.advertise("dtct_area",1);
  PubDepth->image=view_img2.clone();
  dtct_area.publish(PubDepth->toImageMsg());
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
