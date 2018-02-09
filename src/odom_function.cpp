#include"img_prc_cls.h"

	void ImageProcesser::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
		//現在のodometryを取得
		double r,p,y;//一時的に格納するための変数
		tf::Quaternion quat(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
		tf::Matrix3x3(quat).getRPY(r,p,y);
		//set
		position_x=msg->pose.pose.position.x;
		position_y=msg->pose.pose.position.y;
		position_z=msg->pose.pose.position.z;
		roll=r;
		pitch=p;
		yaw=y;
	}
	void ImageProcesser::wheelodom_callback(const obst_avoid::wheel_msg::ConstPtr& msg){
		vr=msg->vel_r;
		vl=msg->vel_l;
	}
	void ImageProcesser::setwodom(void){
		wodom_queue.callOne(ros::WallDuration(0.001));
	}
	//set odometry
	void ImageProcesser::setodom(void){
		if(isOdom()){
			//---get previous time
			getprevtime();
			//get time
			gettime();
			//culculation dt
			culcdt();
			//---一つ前のodometryを格納
			setPrevodom();
			//現在のodometryを格納
			setparam();
			//set delta r,p,y
			setdpose();
			//set global odometry
			setglobalodom();
			bool front=pose_detection(position_x,position_y,prev_yaw);
//			std::cout<<"front:"<<front<<"\n";
			if(!front)
				dxsignchange();
			setdzdx();
//			std::cout<<"dz:"<<dz<<"\n";
		}
		else{
			//現在のodometryを格納
			setparam();
			setodomrcvd();
			//get time
			gettime();
		}
	}
//set x,y,z,r,p,y
	void ImageProcesser::setparam(void){
		odom_queue.callOne(ros::WallDuration(1));
	}
	//set previous odometry
	void ImageProcesser::setPrevodom(void){
		prev_position_x=position_x;
		prev_position_y=position_y;
		prev_position_z=position_z;
		prev_roll=roll;
		prev_pitch=pitch;
		prev_yaw=yaw;
	}
//set delta r,p,y
	void ImageProcesser::setdpose(void){
		pw=w;
		pdyaw=dyaw;
		droll=roll-prev_roll;
		dpitch=pitch-prev_pitch;
		if(prev_yaw>0&&yaw<0&&prev_yaw>PI/2)
			dyaw=(yaw+2*PI)-prev_yaw;
		else if(prev_yaw<0&&yaw>0&&yaw>PI/2)
			dyaw=(yaw-2*PI)-prev_yaw;
		else
			dyaw=yaw-prev_yaw;
//LPF
		w=dyaw/dt;
/*		if(std::abs(w_w)>=0.01){
			T=2*PI*d/std::abs(w_w);
			w=(T*pw+dt*w)/(T+dt);
			dyaw=w*dt;
		}
*/
		std::cout<<"vr,vl:"<<vr<<","<<vl<<"\n";
		w_v=(vr+vl)/2/1000;
//		w_v=0.5447*w_v*w_v+0.6487*w_v+0.0146;
//		w_w=-(vr-vl)/d;//回転角速度
		w_w=(vr-vl)/d/1000;//回転角速度
		w_dyaw=w_w*dt;//回転角
	}
//set global dx,dy
	void ImageProcesser::setglobalodom(void){
		dr=sqrt((position_x*position_x
			-2*position_x*prev_position_x
			+prev_position_x*prev_position_x)
			+(position_y*position_y
			-2*position_y*prev_position_y
			+prev_position_y*prev_position_y));
		global_dx=-dr*cos(-dyaw);
		global_dy=dr*sin(-dyaw);
//		std::cout<<"global_dx,dy:"<<global_dx<<","<<global_dy<<"\n";
	}
	void ImageProcesser::setdzdx(void){
		double prev_dz=dz;
		double prev_dx=dx;
		double T_odm=dt*5;
		dz=global_dx;//visual odometry z座標
		dx=global_dy;//visual odometry x座標
		dz=(T_odm*prev_dz+dt*dz)/(T_odm+dt);
		dx=(T_odm*prev_dx+dt*dx)/(T_odm+dt);
		
	}
//wheater image exist
	bool ImageProcesser::isOdom(void){
		if(ODOMETRY_RECEIVED)
			return true;
		else
			return false;
	}
//set odometryflag
	void ImageProcesser::setodomrcvd(void){
		ODOMETRY_RECEIVED=true;
	}
//odometry dx's sign change
	void ImageProcesser::dxsignchange(void){
		global_dx=(-global_dx);
	}
//進行の向きを取得
	bool ImageProcesser::pose_detection(double position_x,double position_y,double prev_yaw)
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
		return status;
	}

