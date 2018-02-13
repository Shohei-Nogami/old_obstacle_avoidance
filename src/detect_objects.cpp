#include"ros/ros.h"
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<fstream>
#include <std_msgs/Empty.h>
#include"obst_avoid/point3d.h"
#include"obst_avoid/line_point3d.h"
#include"obst_avoid/sqr_point3d.h"
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
	ros::Time start_time;
public:	
	cv_bridge::CvImagePtr depthimg;
	cv_bridge::CvImagePtr pubdepthimg;
	cv::Mat depth_img;

//	::obst_avoid::point3d p3d;
//	::obst_avoid::line_point3d lp3d;
//	::obst_avoid::sqr_point3d sp3d;

	const double f=350;

	const int width=672;
	const int height=376;
	bool rf=false;
	double prev_time;	//
	double new_time=0;	//
	double dt;
	detect_objects(){
	 	pub_dpt=nh.advertise<obst_avoid::separate_object_msg>("separate_object",1);
		nh1.setCallbackQueue(&depth_queue);
		nh2.setCallbackQueue(&empty_queue);
		sub_depth=nh1.subscribe("/output_dptimage",1,&culc_ave::depth_callback,this);
		sub_empty=nh2.subscribe("/empty_msg",1,&culc_ave::empty_callback,this);
	}
	~detect_objects(){

	}
	void depth_callback(const sensor_msgs::ImageConstPtr& msg)
	{
//		lp3d.line_p3d.clear();
//		sp3d.sqr_p3d.clear();
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
//		int cn=12;
//		cnw=cn*2;
//		cnh=cn;

//		double ave_depth[cnh][cnw];
//		int nan_count,count;

//		std::vector< std::vector<bool> > line_objects[height];
//		std::vector<bool> is_same_obj;
		std::vector< cv::Point2i > line_objects[height];
		cv::Point2i edge_point;
		int count_same_number=0;
//		int left_number=0;
//		int right_number=0;
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
		std::vector<cv::Rect> objects;
		cv::Rect rect_temp;
		std::vector<int> task_height;
		for(int h=0;h<height;h++){
			for(int i=0;i<line_objects[h].size();i++){
				if(line_objects[h][i].x==-1)//skip process
					continue;

				rect_temp.x=line_objects[h][0].x;//set param x,y,widht,height
				rect_temp.y=h;
				rect_temp.width=line_objects[h].y-line_objects[h].x;
				rect_temp.height=0;
				objects.push_back(rect_temp);//insert a initial object
				task_object.push_back(line_objects[h][i]);
				task_height.push_back(h);
				line_objects[h].x=-1;//flag
				for(int j=0;j<task_objects.size();j++){
					if(task_height[j] > 1){
						for(int k=0;k<line_objects[task_height[j]-1;k++){
							if(line_objects[h][k].x==-1)//skip process
								continue;
							if(line_objects[task_height[j]-1][k].x < task_objects[ task_height[j] ][j].x){
								if(line_objects[task_height[j]-1][k].y > task_objects[ task_height[j] ][j].x){ 
									task_object.push_back(line_objects[ task_height[j]-1 ][k]);
									task_height.push_back(task_height[j]-1);
								}
							}
							else{
								if(line_objects[task_height[j]-1][k].x < task_objects[task_height[j] ][j].x+task_objects[ task_height[j] ][j].width){ 
									task_object.push_back(line_objects[ task_height[j]-1 ][k]);
									task_height.push_back(task_height[j]-1);
									
								}
								else
									break;
							}
						}
					}
					if(task_height[j] < width-1){
						if(task_objects[j].x < line_objects[j+1].x){
							if(task_objects[j].x > line_objects[j+1].y){ 
							
							}
						}
						else{
							if(task_objects[j].y < line_objects[j+1].x){ 
							
							}
						}					
					}
				}//end for(task)
			}
		}
		long double sum_depth;
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				count=0;
				nan_count=0;
				sum_depth=0;
				for(int h=(int)(i*height/cnh);h<(int)((i+1)*height/cnh);h++){
					for(int w=(int)(j*width/cnw);w<(int)((j+1)*width/cnw);w++){
						
						if(!std::isnan(depth))
							sum_depth+=depth;
						else
							nan_count++;
						count++;
					}
				}
				if((double)nan_count/count>=0.40)
					sum_depth=0;
				ave_depth[i][j]=sum_depth/(count-nan_count);
				p3d.x=( -((double)j*width/cnw+(double)width/cnw/2)+(double)width/2 )*ave_depth[i][j]/f;
				p3d.y=( -((double)i*height/cnh+(double)height/cnh/2)+(double)height/2 )*ave_depth[i][j]/f;
				p3d.z=ave_depth[i][j];
				lp3d.line_p3d.push_back(p3d);
			}
			sp3d.sqr_p3d.push_back(lp3d);
			lp3d.line_p3d.clear();
		}

/*		double min_depth[cnh][cnw];
		int nan_count,count;
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				count=0;
				nan_count=0;
				for(int h=(int)(i*height/cnh);h<(int)((i+1)*height/cnh);h++){
					for(int w=(int)(j*width/cnw);w<(int)((j+1)*width/cnw);w++){
						double depth=depth_img.at<float>(h,w);
						if(!std::isnan(depth)){
							if(count==0)
								min_depth[i][j]=depth;
							else if(min_depth[i][j]>depth)
								min_depth[i][j]=depth;
						}
						else
							nan_count++;
						count++;
					}
				}
				p3d.x=( -((double)j*width/cnw+(double)width/cnw/2)+(double)width/2 )*min_depth[i][j]/f;
				p3d.y=( -((double)i*height/cnh+(double)height/cnh/2)+(double)height/2 )*min_depth[i][j]/f;
				p3d.z=min_depth[i][j];
				lp3d.line_p3d.push_back(p3d);
			}
			sp3d.sqr_p3d.push_back(lp3d);
			lp3d.line_p3d.clear();
		}
*/

	}
	void empty_callback(const std_msgs::Empty& msg){
		rf=true;
	}
	void depthcall(void){
			depth_queue.callOne(ros::WallDuration(1));
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
	void pub3d(void){
		pub_dpt.publish(sp3d);	
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
		ros::init(argc,argv,"culc_area3d");

		culc_ave prc;
//		image_transport::ImageTransport it(nh);
		prc.setstarttime();
		while(ros::ok()){
//			ROS_INFO("prc.depthcall();");			
			prc.depthcall();
//			ROS_INFO("prc.depthcall();");
			if(!prc.isdepth())
				continue;
			//
			prc.culc_prc();
			prc.initrf();
//			ROS_INFO("prc.pub3d()");
			prc.pub3d();
//			ROS_INFO("prc.pub3d(),prc.emptycall();");
//			while(ros::ok()&&!prc.isrf()){
				prc.emptycall();
	//		}
//			ROS_INFO("prc.emptycall();");
/*			if(prc.isnew_time()){
				prc.getprevtime();
				prc.gettime();
				prc.culcdt();
//				prc.print_dt();
			}
			else 
				prc.gettime();
*/			//publish 
			/*cv_bridge::CvImagePtr pubdepthimg(new cv_bridge::CvImage);
		pubdepthimg->encoding=sensor_msgs::image_encodings::MONO8;
		pubdepthimg->image=depth_img.clone();
			pub_dpt.publish(pubdepthimg->toImageMsg());*/
		}
		return 0;
	}
