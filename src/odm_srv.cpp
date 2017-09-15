#include"image_process_zed_function.h"

bool pose_detection(double position_x,double position_y,double prev_yaw);

bool odm_srv(const nav_msgs::Odometry::ConstPtr& odm){

//---メソッドで処理
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

	bool front=pose_detection(position_x,position_y,prev_yaw);
	if(!front)
		visual_dy=(-vidsual_dy);


	return true;
}

bool pose_detection(double position_x,double position_y,double prev_yaw)
{
	bool status=true;
	if(-prev_yaw<(PI/2.0)&&-prev_yaw>0&&
		(-(position_y-prev_position_y)>0||(position_x-prev_position_x)>0)){
		if((-prev_yaw)-atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
			status=false;
	}
	if(-prev_yaw>-(PI/2.0)&&-prev_yaw<0&&
		(-(position_y-prev_position_y)<0||(position_x-prev_position_x)>0)){
		if(-(-prev_yaw)+atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
			status=false;
	}
	if(-prev_yaw>-(PI)&&-prev_yaw<-(PI/2.0)&&
		(-(position_y-prev_position_y)<0||(position_x-prev_position_x)<0)){
		if(PI+(-prev_yaw)-atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
			status=false;
	}
	if(-prev_yaw<(PI)&&-prev_yaw>(PI/2.0)&&
	(-(position_y-prev_position_y)>0||(position_x-prev_position_x)<0)){
		if(PI-(-prev_yaw)+atan(
			(position_x-prev_position_x)/(-(position_y-prev_position_y)))<PI/2.0)
			status=false;
	}
	if(-prev_yaw==0&&(position_x-prev_position_x)>0)
		status=false;
	
	if(-prev_yaw==PI/2.0&&-(position_y-prev_position_y)>0)
		status=false;
	
	if(-prev_yaw==-PI/2.0&&-(position_y-prev_position_y)<0)
		status=false;

	if((-prev_yaw==-PI||-prev_yaw==PI)&&(position_x-prev_position_x)<0)
		status=false;

}





