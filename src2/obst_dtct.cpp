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
image_transport::Publisher fileter_area;
cv_bridge::CvImagePtr depthimg;
int width=672;
int height=376;
bool flag=true;
int threshold=20;
//int threshold_y=30;
cv::Mat depth_img;
//ros::CallbackQueue depth_queue;

//depth_queue.callOne(ros::WallDuration(1));
//subscribe options 

   ros::SubscribeOptions depth_option; 




void lng_dir_approx(const int& i,const int& j,const int& count,const int& threshold,const float& depth1,const float& depth2);

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{


	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	depthimg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//BGR8);

	cv::Mat edit_img = cv::Mat::zeros(cv::Size(width,height),CV_32FC1);
	depth_img=depthimg->image.clone();
	float depth;
	float max_distance=10.0;
	cv::Mat view_img1 = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);
	cv::Mat view_img2 = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);
	cv::Mat view_img3 = cv::Mat::zeros(cv::Size(width,height),CV_8UC3);

	cv::Vec3b color;
	cv::Vec3b white_color;
	white_color.val[0]=255;
	white_color.val[1]=255;
	white_color.val[2]=255;

	for(int i=0;i<height;i++){
		for(int j=0,count=0;j<width;j++){
			if(std::isinf(depth_img.at<float>(i,j))||std::isnan(depth_img.at<float>(i,j)))
				depth_img.at<float>(i,j)=0;
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
			if(depth!=0 && depth_img.at<float>(i,j+1)==0){
				depth1=depth;
				count++;
			}

			if(depth==0){
				//|nan|nan|の区間
				if(depth_img.at<float>(i,j+1)==0)
					count++;
				//|nan|val|のとき
				else{
					depth2=depth_img.at<float>(i,j+1);
					//左端がnanのとき
					if(depth1==0){
						//if(threshold<=count)
						//	lng_dir_approx(i,j,count,threshold,depth1,depth2);
						//else{
							for(int k=0;k<count+1;k++)
								depth_img.at<float>(i,j-k)=depth2;
						//}
					}
					else{
						//if(threshold<=count)
						//	lng_dir_approx(i,j,count,threshold,depth1,depth2);
						//else{						
							for(int k=0;k<count;k++)
								depth_img.at<float>(i,j-k)=depth2-(depth2-depth1)/(count+1)*(k+1);
						//}
					//ROS_INFO("nan|val|:nancount=%d",count);
					}
					count=0;
				}
			}
		//右端がnanのとき
			if(j==(width-1)-1 &&depth_img.at<float>(i,j+1)==0){
				//if(threshold<=count)
				//	lng_dir_approx(i,j,count,threshold,depth1,depth2);
				//else{				
					if(depth1==0)
						std::cout<<"all nan:count("<<count<<")\n";
				
				for(int k=0;k<count;k++)
					depth_img.at<float>(i,j+1-k)=depth1;
				//ROS_INFO("val|nan|nan|:nancount=%d",count);
				count=0;
				//}
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
		flag=false;
	if(flag){
 	 	std::ofstream ofs("approx_depth_graph.csv",std::ios::app);	
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
		flag=false;
	}
//	std::cout<<"(50,350):"<<depth_img.at<float>(350,50)<<"\n";
	cv_bridge::CvImagePtr PubDepth2(new cv_bridge::CvImage);
	PubDepth2->encoding=sensor_msgs::image_encodings::TYPE_32FC1;
	approx_area=it.advertise("approx_area",1);
	PubDepth2->image=depth_img.clone();
	approx_area.publish(PubDepth2->toImageMsg());

//filter

}

void lng_dir_approx(const int& i,const int& j,const int& count,const int& threshold,const float& depth1,const float& depth2){
//std::cout<<"called function\n";
/*
if threshold < count
begin_j=j+1-count
end_j=j
//nanの始点から終点まで
for l=begin_j l<=end_j l++
//一番上の行のとき
if i==0
//上の値は左右のnanから求めた線形近似値を使用
ab_p= (depth2-(depth2-depth1)/(count+1)*(l+1)) 
//2行目からは一つ上の値を使用
else
ab_p=img(i-1,j)
if i==height-1
un_p= (depth2-(depth2-depth1)/(count+1)*(l+1)) 
else
//下のnanではない値を探索
int m
for m=1 ;; m++
if !std::isnan(img(i+m,l))
break
un_p=img(i+m,j)
//縦方向の探索範囲が閾値を以上なら
if 1+m > threshold 
//上下左右4点の平均値を使用
img(i,l)=( (depth2-(depth2-depth1)/(count+1)*(l+1)) + (ab_p-(ab_p-un_p)/(m+1)*1)) /2.0
//上下の値の線形近似を使用
else
img(i,l)=ab_p-(ab_p-un_p)/(m+1)*1 
*/
	int begin_j=j+1-count;
	int end_j=j;
	float ab_p,un_p;
	//nanの始点から終点まで
	for(int l=begin_j;l<=end_j;l++){
	//std::cout<<"(i,j,l):"<<i<<","<<j<<","<<l<<std::endl;
		//一番上の行のとき
		if(i==0){
	//std::cout<<"i==0"<<std::endl;
			if(depth1!=0&&depth2!=0)
				ab_p= (depth2-(depth2-depth1)/(count+1)*(l-j+1));
			else if(depth1==0)
				ab_p= depth2;
			else
				ab_p= depth1;
		}
		//2行目からは一つ上の値を使用
		else
			ab_p=depth_img.at<float>(i-1,j);
		//一番下の行のとき
		if(i==height-1){
	//std::cout<<"i==height-1"<<std::endl;			
			if(depth1!=0&&depth2!=0)
				un_p= (depth2-(depth2-depth1)/(count+1)*(l-j+1));
			else if(depth1==0)
				un_p= depth2;
			else
				un_p= depth1;
			depth_img.at<float>(i,l)=(ab_p+un_p)/2.0;
		}
		else{
	//std::cout<<"else"<<std::endl;
			//下のnanではない値を探索
			int m;
			for (m=1;;m++){
				if(i+m>=height){
					un_p=ab_p;//
					break;
				}
				if(depth_img.at<float>(i+m,l)!=0){
					un_p=depth_img.at<float>(i+m,l);
					break;
				}
			}
//			un_p=depth_img.at<float>(i+m,l);
			//縦方向の探索範囲が閾値を以上なら
			if(1+m>threshold && depth1!=0 && depth2!=0){
				//std::cout<<"1+m>threshold && depth1!=0 && depth2!=0"<<std::endl;
				//上下左右の線形近似で得られた2点の平均値を使用
				depth_img.at<float>(i,l)=( (depth2-(depth2-depth1)/(count+1)*(l-j+1)) + (ab_p-(ab_p-un_p)/(m+1)*1)) /2.0;
			}
				//上下の値の線形近似を使用
			else{
				//std::cout<<"else"<<std::endl;
				depth_img.at<float>(i,l)=ab_p-(ab_p-un_p)/(m)*1;
				if(depth_img.at<float>(i,l)>=10)
					ROS_INFO("(i,j,l,m,ab,un,depth):(%d,%d,%d,%d,%f,%f,%f)",i,j,l,m,ab_p,un_p,depth_img.at<float>(i,l));
			}
		}
	}
	for(int i=0;i<height;i++){
		for(int j=0;j<width;j++){
			if(std::isnan(depth_img.at<float>(i,j)))
				std::cout<<"nan!\n!";
		}
	}
	//std::cout<<"end function\n";
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"obstacle_detection");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
//	nh.setCallbackQueue(&depth_queue);
	sub_depth=it.subscribe("/zed/depth/depth_registered",1,
		&depthImageCallback);
	ros::spin();
	return 0;
}
