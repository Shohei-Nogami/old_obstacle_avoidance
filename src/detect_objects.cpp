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
	//depth image after filter process
	static const int ksize=5;//filter param
	cv::Mat depth_image;
	// rectangle and objects for publish
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
	//filter process	
	//--median
	const int MEDIAN_FILTER=1;
	bool use_cv_function=false;
	std::vector<float> depth_median;
	//--average
	const int AVERAGE_FILTER=2;
	int depth_sum;
	int depth_element_num;
	//for detect process
	std::vector< cv::Point2i > task_objects;

	//------for debug program------
	//debug switch flag
	const bool debug_switch_flag=true;
	// image treansport for publish detected image
	ros::NodeHandle nh4;
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
		:it(nh4)	
	{		
	 	pub_detected_objects=nh.advertise<obst_avoid::detected_objects>("detected_objects",1);
		nh1.setCallbackQueue(&depth_queue);
		nh2.setCallbackQueue(&empty_queue);

		sub_empty=nh2.subscribe("/empty_msg",1,&detect_objects::empty_callback,this);
		if(!debug_switch_flag){
			sub_depth=nh1.subscribe("/output_dptimage",1,&detect_objects::depth_callback,this);
		}
		else{
			sub_depth=nh1.subscribe("/zed/depth/depth_registered",1,&detect_objects::depth_callback,this);

			pub_detected_image=it.advertise("detected_image",1);
			nh3.setCallbackQueue(&original_image_queue);
			sub_original_image=nh3.subscribe("/zed/left/image_rect_color",1,&detect_objects::original_image_callback,this);
		}
		task_objects.reserve(width/ksize*height/ksize);
		objects.rect.reserve(width/ksize*height/ksize);
		depth_median.reserve(ksize*ksize);
		//create filter image
		cv::Mat m = cv::Mat::zeros(cv::Size(width/ksize,height/ksize), CV_32FC1);
		depth_image=m.clone();
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

	void filter_process(void){
		int FILTER_TYPE=MEDIAN_FILTER;//AVERAGE_FILTER;
		if(FILTER_TYPE==MEDIAN_FILTER){
//----median filter
			//use opencv function
			if(use_cv_function){
				cv::medianBlur(depth_img,depth_img,ksize);
				for(int h=0;h<height/ksize;h++){
					for(int w=0;w<width/ksize;w++){
						depth_image.at<float>(h,w)=depth_img.at<float>(h*ksize,w*ksize);
					}
				}
			}//end if
			//without to use opencv function
			else{//self made
				for(int h=0;h<height/ksize;h++){
					for(int w=0;w<width/ksize;w++){
						for(int l=-ksize/2;l<=ksize/2;l++){
							for(int m=-ksize/2;l<=ksize/2;l++){
								if(std::isnan(depth_img.at<float>(h*ksize+l,w*ksize+m))
									||std::isinf(depth_img.at<float>(h*ksize+l,w*ksize+m)) 
									||h+l<0||height/ksize<=h+l
									||w+m<0||width/ksize<=w+m ){
									continue;
								}
								depth_median.push_back(depth_img.at<float>(h*ksize+l,w*ksize+m) );
							}
						}
						std::sort(depth_median.begin(),depth_median.end());
						if((int)depth_median.size()){
							depth_image.at<float>(h,w)=depth_median[(int)depth_median.size()/2];
						}
						else{
							depth_image.at<float>(h,w)=0;						
						}
						depth_median.clear();
					}
				}
			}//end else
		}
		else if(FILTER_TYPE==AVERAGE_FILTER){
			for(int h=0;h<height/ksize;h++){
				for(int w=0;w<width/ksize;w++){
					depth_sum=0;
					depth_element_num=0;
					for(int l=-ksize/2;l<=ksize/2;l++){
						for(int m=-ksize/2;l<=ksize/2;l++){
							if(std::isnan(depth_img.at<float>(h*ksize+l,w*ksize+m))
								||std::isinf(depth_img.at<float>(h*ksize+l,w*ksize+m))
								||h+l<0||height/ksize<=h+l
								||w+m<0||width/ksize<=w+m ){
								continue;
							}
							depth_sum+=depth_img.at<float>(h*ksize+l,w*ksize+m);
							depth_element_num++;
						}
					}
					depth_image.at<float>(h,w)=depth_sum/depth_element_num;						
				}
			}
		}
	}
	void debug_filter(void){
	//debug filter
		int FILTER_TYPE=MEDIAN_FILTER;//AVERAGE_FILTER;
		if(FILTER_TYPE==MEDIAN_FILTER){
//----median filter
			//use opencv function
			if(use_cv_function){
				cv::medianBlur(depth_img,depth_img,ksize);
				for(int h=0;h<height/ksize;h++){
					for(int w=0;w<width/ksize;w++){
						depth_image.at<float>(h,w)=depth_img.at<float>(h*ksize,w*ksize);
					}
				}
			}//end if
			//without to use opencv function
			else{//self made
				for(int h=0;h<height/ksize;h++){
					for(int w=0;w<width/ksize;w++){
						for(int l=-ksize/2;l<=ksize/2;l++){
							for(int m=-ksize/2;l<=ksize/2;l++){
								if(std::isnan(depth_img.at<float>(h*ksize+l,w*ksize+m))
									||std::isinf(depth_img.at<float>(h*ksize+l,w*ksize+m)) 
									||h+l<0||height/ksize<=h+l
									||w+m<0||width/ksize<=w+m ){
									continue;
								}
								depth_median.push_back(depth_img.at<float>(h*ksize+l,w*ksize+m) );
							}
						}
						std::sort(depth_median.begin(),depth_median.end());
						if((int)depth_median.size()){
							depth_image.at<float>(h,w)=depth_median[(int)depth_median.size()/2];
						}
						else{
							depth_image.at<float>(h,w)=0;						
						}
						depth_median.clear();
					}
				}
			}//end else
		}
		else if(FILTER_TYPE==AVERAGE_FILTER){
			for(int h=0;h<height/ksize;h++){
				for(int w=0;w<width/ksize;w++){
					depth_sum=0;
					depth_element_num=0;
					for(int l=-ksize/2;l<=ksize/2;l++){
						for(int m=-ksize/2;l<=ksize/2;l++){
							if(std::isnan(depth_img.at<float>(h*ksize+l,w*ksize+m))
								||std::isinf(depth_img.at<float>(h*ksize+l,w*ksize+m))
								||h+l<0||height/ksize<=h+l
								||w+m<0||width/ksize<=w+m ){
								continue;
							}
							depth_sum+=depth_img.at<float>(h*ksize+l,w*ksize+m);
							depth_element_num++;
						}
					}
					depth_image.at<float>(h,w)=depth_sum/depth_element_num;						
				}
			}
		}
		cv_bridge::CvImagePtr detectd_cvimage(new cv_bridge::CvImage);
		detectd_cvimage->encoding=sensor_msgs::image_encodings::TYPE_32FC1;
		detectd_cvimage->image=depth_image.clone();
		pub_detected_image.publish(detectd_cvimage->toImageMsg());
	}

	void detect_process(void){
		
		int search_range=1;
		bool searched_flag[height/ksize][width/ksize];
		//initialize searched_flag
		for(int h=0;h<height/ksize;h++){
			for(int w=0;w<width/ksize;w++){
				searched_flag[h][w]=false;
			}
		}
		//initialize 
		objects.rect.clear();
		//detect process
		::obst_avoid::points tl,br;
		::obst_avoid::rectangle rect_temp;
		cv::Point2i temp;
		for(int h=0+search_range;h<height/ksize-search_range;h++){
			for(int w=0+search_range;w<width/ksize-search_range;w++){
//				std::cout<<"depth_image.at<float>(h,w):"<<depth_image.at<float>(h,w)<<"\n";
//				std::cout<<"w,width/ksize-search_range:"<<w<<","<<width/ksize-search_range<<"\n";
//				std::cout<<"h<height/ksize-search_range:"<<h<<","<<height/ksize-search_range<<"\n";
				if(searched_flag[h][w]||depth_image.at<float>(h,w)==0){
					continue;
				}
				task_objects.clear();
				tl.x=w;
				tl.y=h;
				br.x=w;
				br.y=h;
//				cv::Point2i temp;
				temp.x=w;
				temp.y=h;
//				std::cout<<"temp:"<<temp<<"\n";
				task_objects.push_back(temp);
				searched_flag[h][w]=true;
//				std::cout<<"task_objects.size:"<<task_objects.size()<<"\n";
				//search process
				for(int i=0;i<task_objects.size();i++){
					float depth_0=depth_image.at<float>(task_objects[i].y,task_objects[i].x);
					for(int l=-search_range;l<=search_range;l++){
						for(int m=-search_range;m<=search_range;m++){

							if(searched_flag[task_objects[i].y+l][task_objects[i].x+m])
								continue;
//							if(l==m)
//								continue;
							float depth_i=depth_image.at<float>(task_objects[i].y+l,task_objects[i].x+m);
							if(depth_i==0)
								continue;
							if(0 > task_objects[i].y+l || task_objects[i].y+l > height/ksize
								||0 > task_objects[i].x+m || task_objects[i].x+m > width/ksize)
								continue;
							if(abs(depth_i-depth_0)<=0.02){
								searched_flag[task_objects[i].y+l][task_objects[i].x+m]=true;
								temp.x=task_objects[i].x+m;
								temp.y=task_objects[i].y+l;
								task_objects.push_back(temp);
							}
//							std::cout<<"task_objects[i]:"<<task_objects[i]<<"\n";
						}//m
//						std::cout<<"task_objects:"<<task_objects.size()<<"\n";
					}//l
					if(tl.x<task_objects[i].x)
						tl.x=task_objects[i].x;
					else if(br.x>task_objects[i].x)
						br.x=task_objects[i].x;
					if(tl.y<task_objects[i].y)
						tl.y=task_objects[i].y;
					else if(br.y>task_objects[i].y)
						br.y=task_objects[i].y;	

				}//task

				//debug
				ROS_INFO("task_object.size:%d",(int)task_objects.size());

				if((int)task_objects.size()>10){
					rect_temp.tl.x=tl.x*ksize;
					rect_temp.tl.y=tl.y*ksize;
					rect_temp.br.x=br.x*ksize;
					rect_temp.br.y=br.y*ksize;
					//insert a rectangle
					objects.rect.push_back(rect_temp);
				}
				
			}
		}

		std::cout<<"end detect process\n";
		//publish detected_objects		
		if((int)objects.rect.size()){
			pub_detected_objects.publish(objects);
		}
		std::cout<<"end publish process\n";
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

		detect_objects prc;
		prc.setstarttime();
		while(ros::ok()){
//			std::cout<<"while start\n";
			if(prc.is_debug_flag()){
				prc.original_image_call();
			}
//			std::cout<<"prev depth_call\n";
			prc.depthcall();
//			std::cout<<"prev isdepth\n";
			if(!prc.isdepth())
				continue;
//			prc.debug_filter();
			prc.filter_process();
//			std::cout<<"a\n";
			prc.detect_process();
			if(!prc.is_debug_flag()){
				prc.initrf();
				prc.emptycall();
			}
			else{
				prc.publish_detected_image();
			}

		}
		return 0;
	}
