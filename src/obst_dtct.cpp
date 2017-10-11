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
image_transport::Publisher dtct_area;
image_transport::Publisher approx_area;
cv_bridge::CvImagePtr depthimg;
int width=672;
int height=376;
bool flag=true;
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  depthimg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//BGR8);
  cv::Mat depth_img;
  cv::Mat edit_img = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);
  depth_img=depthimg->image.clone();
  float depth;
  float max_distance=10.0;
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
 
/*  float depth1=0,depth2=0;
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
*/	
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
      //各行ごとを白く塗りつぶす 完了
    }
  }


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

//  cv_bridge::CvImagePtr PubDepth(new cv_bridge::CvImage);
//  PubDepth->encoding=sensor_msgs::image_encodings::BGR8;
  dtct_area=it.advertise("dtct_area",1);
  PubDepth->image=view_img2.clone();
  dtct_area.publish(PubDepth->toImageMsg());
*/
  for(int i=0;i<height;i++){
      for(int j=0,count=0;j<width;j++){
	if(std::isinf(depth_img.at<float>(i,j)))
		depth_img.at<float>(i,j)=max_distance;
      }
  }
  float depth1,depth2;
    for(int i=0;i<height;i++){
      depth=0;
      depth1=0;
      depth2=0;
      for(int j=0,count=0;j<width-1;j++){
        depth=depth_img.at<float>(i,j);
        //|val|nan|のとき
        if(!std::isnan(depth) && std::isnan(depth_img.at<float>(i,j+1))){
          depth1=depth;
          count++;
        }
        
        if(std::isnan(depth)){
          //|nan|nan|の区間
          if(std::isnan(depth_img.at<float>(i,j+1)))
            count++;
          //|nan|val|のとき
          else{
            depth2=depth_img.at<float>(i,j+1);
            //左端がnanのとき
            if(depth1==0){
              for(int k=0;k<count+1;k++)
                depth_img.at<float>(i,j-k)=depth2;
            }
            else{
              for(int k=0;k<count;k++)
                depth_img.at<float>(i,j-k)=depth2-(depth2-depth1)/(count+1)*(k+1);
              ROS_INFO("nan|val|:nancount=%d",count);
            }
            count=0;
          }
        }
        //右端がnanのとき
        if(j==(width-1)-1 &&std::isnan(depth_img.at<float>(i,j+1))){
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
	if(std::isnan(depth_img.at<float>(i,j)))
		nc++;
      }
  }
  ROS_INFO("nan:%d",nc);
  if(flag){
 	 std::ofstream ofs("approx_depth_graph.csv",std::ios::app);	
	  for(int i=0;i<height;i++){
 	     for(int j=0;j<width;j++){
	      ofs << ",";
		for(int j=0;j<width;j++)
		    ofs << j << ",";
		ofs << std::endl;
		for(int i=0;i<height;i++){
		    ofs << i << ",";
		    for(int j=0;j<width;j++)
		        ofs << depth_img.at<float>(i,j) << ",";
		    ofs << std::endl;
		}
	      }
	  }
	  flag=false;
  }
  cv_bridge::CvImagePtr PubDepth2(new cv_bridge::CvImage);
  PubDepth2->encoding=sensor_msgs::image_encodings::TYPE_32FC1;
  approx_area=it.advertise("approx_area",1);
  PubDepth2->image=depth_img.clone();
  approx_area.publish(PubDepth2->toImageMsg());

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
