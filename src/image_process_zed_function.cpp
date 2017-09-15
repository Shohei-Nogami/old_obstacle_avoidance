#include"image_process_zed_function.h"

ImageProcesser::ImageProcesser()//
	:it(nh),PROCESS_ORDER(1),DEPTH_RECEIVED(false),ODOMETRY_RECEIVED(false),width(672),height(376),max_value_x(0),min_value_x(100),max_value_y(0),min_value_y(100)//width(2560),height(720)//,width(3840),height(1080)
{
	start_time = ros::Time::now();
    pub=it.advertise("output_image",1);
    pub_Limg=it.advertise("output_Limage",1);
    pub_Lmsk=it.advertise("output_Mskimage",1);
	sub_Limg=it.subscribe("/zed/left/image_rect_color",1,
       &ImageProcesser::imageCallback,this);
	sub_depth=it.subscribe("/zed/depth/depth_registered",1,
	   	&ImageProcesser::depthImageCallback,this);
	sub_odom=nh.subscribe("/zed/odom",1,&ImageProcesser::OdometryCallback,this);
	ros::spin();
}

//矢印描写用関数
void ImageProcesser::cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color)
{
    int thickness=4;
    int lineType=8;
    int shift=0;
    cv::line(*img,pt1,pt2,color,thickness,lineType,shift);
    float vx = (float)(pt2.x - pt1.x);
    float vy = (float)(pt2.y - pt1.y);
    float v = sqrt( vx*vx + vy*vy );
    float ux = vx / v;
    float uy = vy / v;
//矢印の幅の部分
    float w=5,h=10;
    cv::Point2i ptl,ptr;
    ptl.x = (int)((float)pt2.x - uy*w - ux*h);
    ptl.y = (int)((float)pt2.y + ux*w - uy*h);
    ptr.x = (int)((float)pt2.x + uy*w - ux*h);
    ptr.y = (int)((float)pt2.y - ux*w - uy*h);
//矢印の先端を描画する
//--例外処理(!(v==0))
	if(!(v==0)){
    	cv::line(*img,pt2,ptl,color,thickness,lineType,shift);
    	cv::line(*img,pt2,ptr,color,thickness,lineType,shift);
	}
}
//depth画像をprivate変数に格納
void ImageProcesser::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//debug
	std::cout<<"2:depth recerve\n";

		if(!(PROCESS_ORDER==2)){
			return ;
		}
//	ROS_INFO("PROCESS_ORDER is %d.\n",PROCESS_ORDER);
	PROCESS_ORDER=3;
//	std::cout<<"depth received\n";
    try{
            depthimg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//BGR8);
			DEPTH_RECEIVED=true;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
        msg->encoding.c_str());
        return ;
    }
}

void ImageProcesser::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
//debug
	std::cout<<"3:odom recerve\n";

		if(!(PROCESS_ORDER==3)){//!(PROCESS&&DEPTH)->not run
			return ;	
		}
	ros::Duration process_time = ros::Time::now()-start_time;
	PROCESS_ORDER=1;
    if(ODOMETRY_RECEIVED){
		prev_time=new_time;
		new_time = process_time.toSec();

        prev_position_x=position_x;
        prev_position_y=position_y;
        prev_position_z=position_z;
        prev_roll=roll;
        prev_pitch=pitch;
        prev_yaw=yaw;

		position_x=msg->pose.pose.position.x;
		position_y=msg->pose.pose.position.y;
		position_z=msg->pose.pose.position.z;
				
		tf::Quaternion quat(
					msg->pose.pose.orientation.x,
					msg->pose.pose.orientation.y,
					msg->pose.pose.orientation.z,
					msg->pose.pose.orientation.w);

		tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
		droll=roll-prev_roll;
		dpitch=pitch-prev_pitch;
		dyaw=yaw-prev_yaw;

		double r=sqrt((position_x*position_x
				-2*position_x*prev_position_x
				+prev_position_x*prev_position_x)
				+(position_y*position_y
				-2*position_y*prev_position_y
				+prev_position_y*prev_position_y));
		
		visual_dx=-r*cos(-dyaw);
		visual_dy=r*sin(-dyaw);

//change the sign of visual_dy
		if(-prev_yaw<(PI/2.0)&&-prev_yaw>0&&
			(-(position_y-prev_position_y)>0||(position_x-prev_position_x)>0)){
			if((-prev_yaw)-atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				visual_dx=-visual_dx;
		}
		if(-prev_yaw>-(PI/2.0)&&-prev_yaw<0&&
			(-(position_y-prev_position_y)<0||(position_x-prev_position_x)>0)){
			if(-(-prev_yaw)+atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				visual_dx=-visual_dx;

		}
		if(-prev_yaw>-(PI)&&-prev_yaw<-(PI/2.0)&&
			(-(position_y-prev_position_y)<0||(position_x-prev_position_x)<0)){
			if(PI+(-prev_yaw)-atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				visual_dx=-visual_dx;
		}	
		if(-prev_yaw<(PI)&&-prev_yaw>(PI/2.0)&&
			(-(position_y-prev_position_y)>0||(position_x-prev_position_x)<0)){
			if(PI-(-prev_yaw)+atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
				visual_dx=-visual_dx;
		}
		if(-prev_yaw==0&&(position_x-prev_position_x)>0)
			visual_dx=-visual_dx;
			
		if(-prev_yaw==PI/2.0&&-(position_y-prev_position_y)>0)
			visual_dx=-visual_dx;
		
		if(-prev_yaw==-PI/2.0&&-(position_y-prev_position_y)<0)
			visual_dx=-visual_dx;

		if((-prev_yaw==-PI||-prev_yaw==PI)&&(position_x-prev_position_x)<0)
			visual_dx=-visual_dx;
		//next Process
		ImageProcesser::imageProcess();
    }
    else{
		prev_time=process_time.toSec();
        prev_position_x=msg->pose.pose.position.x;
        prev_position_y=msg->pose.pose.position.y;
        prev_position_z=msg->pose.pose.position.z;
		tf::Quaternion quat(
					msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
					msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
		tf::Matrix3x3(quat).getRPY(prev_roll,prev_pitch,prev_yaw);
		ODOMETRY_RECEIVED=true;
    }
}
/*
void ImageProcesser::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if(!(PROCESS_ORDER==1))
		return ;
	PROCESS_ORDER=2;
	try{
		org_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
		msg->encoding.c_str());
		return ;
	}

}
*/

