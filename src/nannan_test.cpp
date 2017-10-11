#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>


int width=10;
int height=2;


int main(int argc,char **argv)
{
	ros::init(argc,argv,"apporx_test");
	cv::Mat depth_img= cv::Mat::zeros(cv::Size(width,height),CV_32FC1);
	for(int i=0;i<height;i++){
      for(int j=0;j<width;j++){
		depth_img.at<float>(i,j)=10;
	  }
	}
	depth_img.at<float>(0,4)=0;
	depth_img.at<float>(0,5)=0;
	depth_img.at<float>(0,6)=11;
	depth_img.at<float>(0,7)=0;
	depth_img.at<float>(0,8)=0;
	depth_img.at<float>(0,9)=8;
	depth_img.at<float>(1,2)=1;
	depth_img.at<float>(1,3)=0;
	depth_img.at<float>(1,4)=0;
    for(int i=0;i<height;i++){
      for(int j=0;j<width-1;j++){
      	ROS_INFO("(%d,%d):(%f,%f)",i,j,
		depth_img.at<float>(i,j),depth_img.at<float>(i,j+1));
	}
    }

  float depth,depth1,depth2;
    for(int i=0;i<height;i++){
      depth=0;
      depth1=0;
      depth2=0;
      for(int j=0,count=0;j<width-1;j++){
        depth=depth_img.at<float>(i,j);
        //|val|nan|のとき
		ROS_INFO("(%d,%d):",i,j);

        if(depth!=0 && depth_img.at<float>(i,j+1)==0){
          depth1=depth;
          count++;
			ROS_INFO("!nan&nan(%d,%d)",j,j+1);
        }
        
        if(depth==0){
          //|nan|nan|の区間
          if(depth_img.at<float>(i,j+1)==0){
            count++;
			ROS_INFO("nan&nan(%d,%d)",j,j+1);
		  }
          //|nan|val|のとき
          else{
			ROS_INFO("nan&!nan(%d,%d)",j,j+1);
            depth2=depth_img.at<float>(i,j+1);
            //左端がnanのとき
            if(depth1==0){
              for(int k=0;k<count+1;k++)
                depth_img.at<float>(i,j-k)=depth2;

            }
            else{
              for(int k=0;k<count;k++)
                depth_img.at<float>(i,j-k)=depth2-(depth2-depth1)/(count+1)*(k+1);

            }
              ROS_INFO("nan|val|:nancount=%d",count);
            count=0;
          }
        }
        //右端がnanのとき
        if(j==(width-1)-1 &&depth_img.at<float>(i,j+1)==0){
          for(int k=0;k<count;k++)
            depth_img.at<float>(i,j+1-k)=depth1;
          ROS_INFO("val|nan|nan|:nancount=%d",count);
          count=0;
        }
      }
    }
  int nc=0;
  for(int i=0;i<height;i++){
      for(int j=0;j<width;j++){
	if(depth_img.at<float>(i,j)==0)
		nc++;
      }
  }
  ROS_INFO("nan:%d",nc);
     for(int i=0;i<height;i++){
      for(int j=0;j<width-1;j++){
      	ROS_INFO("(%d,%d):(%f,%f)",i,j,
		depth_img.at<float>(i,j),depth_img.at<float>(i,j+1));
	}
    }
 return 0;
}
