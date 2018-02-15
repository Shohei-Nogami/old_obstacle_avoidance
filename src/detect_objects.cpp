#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<fstream>
#include <std_msgs/Empty.h>
#include"obst_avoid/points.h"
#include"obst_avoid/rectangle.h"
#include"obst_avoid/detected_objects.h"
#include <ros/callback_queue.h>

class detect_objects
{

	ros::NodeHandle nh,nh1,nh2;
	//subscriber
	ros::Subscriber sub_depth;//DepthImage
	ros::Subscriber sub_empty;
	ros::CallbackQueue depth_queue;
	ros::CallbackQueue empty_queue;
	//subscribe options
	ros::SubscribeOptions depth_option;
	ros::SubscribeOptions empty_option;
	ros::Publisher pub_dpt;
	ros::Publisher pub_detected_objects;
	//start time
	ros::Time start_time;
public:
	//received depth image
	cv_bridge::CvImagePtr depthimg;
	cv::Mat depth_img;
	// rectangle and objects for publish
	::obst_avoid::rectangle rect_temp;
	::obst_avoid::detected_objects objects;
	//camera param
	const double f=350;
	static const int width=672;
	static const int height=376;
	//response_flag
	bool rf=false;
	//time
	double prev_time;	//
	double new_time=0;	//
	double dt;
	//for detect process
	std::vector< cv::Point2i > line_objects[height];
	std::vector< cv::Point2i > task_objects;
	std::vector< int > task_height;
	
	//------for debug program------
	//debug switch flag
	const bool debug_switch_flag=true;
	// image treansport for publish detected image
	image_transport::ImageTransport it;
	//original image
	cv_bridge::CvImagePtr original_image;
	//detectd image
	cv::Mat detectd_image;
	//subsctibe original image
	ros::NodeHandle nh3;
	ros::Subscriber sub_original_image;
	ros::CallbackQueue original_image_queue;
	ros::SubscribeOptions original_image_option;
	image_transport::Publisher pub_detected_image;
	detect_objects()
		:it(nh3)	
	{		
		ROS_INFO("prev define pub and nh1,nh2");
	 	pub_detected_objects=nh.advertise<obst_avoid::detected_objects>("detected_objects",1);
		nh1.setCallbackQueue(&depth_queue);
		nh2.setCallbackQueue(&empty_queue);
		
		sub_empty=nh2.subscribe("/empty_msg",1,&detect_objects::empty_callback,this);
		ROS_INFO("prev sub_empty");
		if(!debug_switch_flag){
			sub_depth=nh1.subscribe("/output_dptimage",1,&detect_objects::depth_callback,this);
		}
		else{
			sub_depth=nh1.subscribe("/zed/depth/depth_registered",1,&detect_objects::depth_callback,this);
			
			pub_detected_image=it.advertise("detected_image",1);
			nh3.setCallbackQueue(&original_image_queue);
			sub_original_image=nh3.subscribe("/zed/left/image_rect_color",1,&detect_objects::original_image_callback,this);
		}
		ROS_INFO("prev reserve");
		//reverve memory of vector 
		for(int i=0;i<height;i++){
			line_objects[i].reserve(width);
		}
		task_height.reserve(width);
		task_objects.reserve(width);
		
	}
	~detect_objects(){
	}
	
	void depth_callback(const sensor_msgs::ImageConstPtr& msg){
		try{
			depthimg= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			depth_img=depthimg->image;
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
			msg->encoding.c_str());
			return ;
		}
	}
	void detect_process(void){
		objects.rect.clear();
		cv::Point2i edge_point;
		int count_same_number=0;
		double previous_depth=depth_img.at<float>(0,0);
		for(int h=0;h<height;h++){
			for(int w=1;w<width;w++){
			  double depth=depth_img.at<float>(h,w);
			  if(abs(depth-previous_depth)<0.10 ){
			    count_same_number++;
			  }
			  else{
			    edge_point.y=w;
			    edge_point.x=w-count_same_number;
			    line_objects[h].push_back(edge_point);
			    count_same_number=0;
			  }
			}
		}

//		std::vector<int> task_height;
		// bool on_next_process_flag;
		// bool under_next_process_flag;
		for(int h=0;h<height;h++){
			for(int i=0;i<line_objects[h].size();i++){
				if(line_objects[h][i].x==-1)//skip process
					continue;
				//initialize task_objects ,task_height
				task_objects.clear();
				task_height.clear();
				task_objects.push_back(line_objects[h][i]);
				task_height.push_back(h);
				//already finished search flag
				line_objects[h][i].x=-1;
				//serch_process
				for(int j=0;j<task_objects.size();j++){
//				  on_next_process_flag=true;
//				  under_next_process_flag=true;
					//when search number < width-1
					if(task_height[j] > 1){// && on_next_process_flag){
						for(int k=0;k<line_objects[task_height[j]-1].size();k++){
							if(line_objects[h][k].x==-1)//skip process
								continue;
							if(line_objects[task_height[j]-1][k].x < task_objects[j].x){
								if(line_objects[task_height[j]-1][k].y > task_objects[j].x){
									task_objects.push_back(line_objects[ task_height[j]-1 ][k]);
									task_height.push_back(task_height[j]-1);
									line_objects[ task_height[j]-1 ][k].x=-1;
								}
							}
							else{
								if(line_objects[task_height[j]-1][k].x < task_objects[j].y){
									task_objects.push_back(line_objects[ task_height[j]-1 ][k]);
									task_height.push_back(task_height[j]-1);
									line_objects[ task_height[j]-1 ][k].x=-1;

								}
								else
									break;
									// on_next_process_flag=false;
							}
						}
					}
					//when search number < width-1
					if(task_height[j] < width-1){// && under_next_process_flag){
						for(int k=0;k<line_objects[task_height[j]-1].size();k++){
							if(line_objects[h][k].x==-1)//skip process
								continue;
							if(task_objects[j].x < line_objects[task_height[j]+1][k].x){
								if(task_objects[j].y > line_objects[task_height[j]+1][k].x){
										task_objects.push_back(line_objects[ task_height[j]+1 ][k]);
										task_height.push_back(task_height[j]+1);
										line_objects[ task_height[j]+1 ][k].x=-1;
								}
							}
							else{
								if(task_objects[j].x < line_objects[task_height[j]+1][k].y){
										task_objects.push_back(line_objects[ task_height[j]+1 ][k]);
										task_height.push_back(task_height[j]+1);
										line_objects[ task_height[j]+1 ][k].x=-1;
								}
								else
									break;
								  // under_next_process_flag=false;
							}
						}
					}

				}//end for(task)
				//culculate rectangle
				::obst_avoid::points tl,br;
				if(task_objects.size()){
				  tl.x=task_objects[0].x;
				  tl.y=task_height[0];
				  br.x=task_objects[0].y;
				  br.y=task_height[0];

				for(int j=1;j<task_objects.size();j++){
				  if(tl.x>task_objects[j].x)
				    tl.x=task_objects[j].x;
				  if(tl.y>task_height[j])
				    tl.y=task_height[j];
				  if(br.x<task_objects[j].x)
				    br.x=task_objects[j].x;
				  if(br.y<task_height[j])
				    br.y=task_height[j];
				}

				rect_temp.tl=tl;
				rect_temp.br=br;
				//insert a rectangle
				objects.rect.push_back(rect_temp);

				}
			}
		}
		//publish detected_objects
		pub_detected_objects.publish(objects);
	}
	bool is_debug_flag(void){
		return debug_switch_flag;
	}
	void original_image_callback(const sensor_msgs::ImageConstPtr& msg){
		try{
			original_image= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			detectd_image=original_image->image;
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
			msg->encoding.c_str());
			return ;
		}
	}
	void publish_detected_image(void){
		for(int i=0;i<objects.rect.size();i++){
			rectangle(detectd_image, cv::Point(objects.rect[i].tl.x,objects.rect[i].tl.y) ,
			 cv::Point(objects.rect[i].br.x,objects.rect[i].br.y) , cv::Scalar(0,200,0), 3, 4);
		}
		cv_bridge::CvImagePtr detectd_cvimage(new cv_bridge::CvImage);
		detectd_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
		detectd_cvimage->image=detectd_image.clone();
		pub_detected_image.publish(detectd_cvimage->toImageMsg());

	}
	void empty_callback(const std_msgs::Empty& msg){
		rf=true;
	}
	void depthcall(void){
			depth_queue.callOne(ros::WallDuration(1));
	}
	void original_image_call(void){
			original_image_queue.callOne(ros::WallDuration(1));
	}
	bool isdepth(void){
		if(depth_img.empty())
			return false;
		else
			return true;
	}
	void initrf(void){
		rf=false;
	}
	bool isrf(void){
		if(rf)
			return true;
		else
			return false;
	}

	void emptycall(void){
		empty_queue.callOne(ros::WallDuration(1));
	}
	void setstarttime(void){
		start_time= ros::Time::now();
	}
	void gettime(void){
		ros::Duration time = ros::Time::now()-start_time;
		new_time=time.toSec();
	}
	void getprevtime(void){
		prev_time=new_time;
	}
	void culcdt(void){
		dt=new_time-prev_time;
	}
	void print_dt(void){
		ROS_INFO("(dt,fps):(%f,%f)",dt,1/dt);
	}
	bool isnew_time(void){
		if(new_time==0)
			return false;
		else
			return true;
	}
};

	int main(int argc,char **argv){
		ros::init(argc,argv,"detect_objects");
		ROS_INFO("process start");
		
		detect_objects prc;
		prc.setstarttime();
		ROS_INFO("prev while");
		while(ros::ok()){
			ROS_INFO("prev if1");
			if(prc.is_debug_flag()){
				prc.original_image_call();
			}
			ROS_INFO("prev depth call");
			prc.depthcall();
			ROS_INFO("prev isdepth");
			if(!prc.isdepth())
				continue;
			ROS_INFO("prev detect_process");
			prc.detect_process();
			ROS_INFO("prev is_debug_flag");
			if(prc.is_debug_flag()){
				prc.initrf();
				prc.emptycall();
			}
			else{
				prc.publish_detected_image();
			}
		}
		return 0;
	}
