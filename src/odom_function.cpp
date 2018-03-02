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
		// R.row(0).col(0) = cos(pitch)*cos(yaw) - sin(roll)*sin(pitch)*sin(yaw);
		// R.row(0).col(1) = -cos(roll)*sin(yaw);
		// R.row(0).col(2) = sin(pitch)*cos(yaw) + sin(roll)*cos(pitch)*sin(yaw);
		// R.row(1).col(0) = cos(pitch)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw);
		// R.row(1).col(1) = cos(roll)*cos(yaw);
		// R.row(1).col(2) = sin(pitch)*sin(yaw) - sin(roll)*cos(pitch)*cos(yaw);
		// R.row(2).col(0) = - cos(roll)*sin(pitch);
		// R.row(2).col(1) = sin(roll);
		// R.row(2).col(2) = cos(roll)*cos(pitch);
		// position_x=msg->pose.pose.position.x;
		// position_y=msg->pose.pose.position.y;
		// position_z=msg->pose.pose.position.z;




		position_x = ( cos(pitch)*cos(yaw) )*msg->pose.pose.position.x
		- ( cos(pitch)*sin(yaw) )*msg->pose.pose.position.y
		+ ( sin(pitch) )*msg->pose.pose.position.z;

		position_y = ( cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw) )*msg->pose.pose.position.x
		+ ( cos(roll)*cos(yaw) - sin(roll)*sin(pitch)*sin(yaw) )*msg->pose.pose.position.y
		- ( sin(roll)*cos(pitch) )*msg->pose.pose.position.z;

		position_z = ( sin(roll)*sin(yaw) - cos(roll)*sin(pitch)*cos(yaw) )*msg->pose.pose.position.x
		+ ( sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw) )*msg->pose.pose.position.y;
		+ ( cos(roll)*cos(pitch) )*msg->pose.pose.position.z;

	}
	void ImageProcesser::wheelodom_callback(const obst_avoid::wheel_msg::ConstPtr& msg){
		vr=msg->vel_r;
		vl=msg->vel_l;
	}
	void ImageProcesser::set_wodom(void){
		wodom_queue.callOne(ros::WallDuration(0.001));
	}
	//set odometry
	void ImageProcesser::set_odom(void){
		if(is_Odom()){
			//---get previous time
			getprevtime();
			//get time
			gettime();
			//culculation dt
			culcdt();
			//---一つ前のodometryを格納
			set_Prevodom();
			//現在のodometryを格納
			set_param();
			//set delta r,p,y
			set_dpose();
			//set global odometry
			set_globalodom();
			bool front=pose_detection(position_x,position_y,prev_yaw);
			if(!front)
				dxsignchange();
			set_dzdx();
		}
		else{
			//現在のodometryを格納
			set_param();
			set_odomrcvd();
			//get time
			gettime();
		}
	}
//set x,y,z,r,p,y
	void ImageProcesser::set_param(void){
		odom_queue.callOne(ros::WallDuration(1));
	}
	//set previous odometry
	void ImageProcesser::set_Prevodom(void){
		prev_position_x=position_x;
		prev_position_y=position_y;
		prev_position_z=position_z;
		prev_roll=roll;
		prev_pitch=pitch;
		prev_yaw=yaw;
	}
//set delta r,p,y
	void ImageProcesser::set_dpose(void){
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

		std::cout<<"vr,vl:"<<vr<<","<<vl<<"\n";
		w_v=(vr+vl)/2/1000;
		w_v=0.5447*w_v*w_v+0.6487*w_v+0.0146;
		w_w=(vr-vl)/d/1000;//回転角速度
		w_dyaw=w_w*dt;//回転角

//LPF
		w=dyaw/dt;
		if(std::abs(w_w)>=0.01){
			T=2*PI*d/std::abs(w_w);
			w=(T*pw+dt*w)/(T+dt);
			dyaw=w*dt;
		}


	}
//set global dx,dy
	void ImageProcesser::set_globalodom(void){
		dr=sqrt((position_x*position_x
			-2*position_x*prev_position_x
			+prev_position_x*prev_position_x)
			+(position_y*position_y
			-2*position_y*prev_position_y
			+prev_position_y*prev_position_y)
			+(position_z*position_z
			-2*position_z*prev_position_z
			+prev_position_z*prev_position_z));
		global_dx=-dr*cos(-dyaw);
		global_dy=dr*sin(-dyaw);
	}
	void ImageProcesser::set_dzdx(void){
		double prev_dz=dz;
		double prev_dx=dx;
		double T_odm=dt*5;
		dz=global_dx;//visual odometry z座標
		dx=global_dy;//visual odometry x座標
		dz=(T_odm*prev_dz+dt*dz)/(T_odm+dt);
		dx=(T_odm*prev_dx+dt*dx)/(T_odm+dt);
		
	}
//wheater image exist
	bool ImageProcesser::is_Odom(void){
		if(ODOMETRY_RECEIVED)
			return true;
		else
			return false;
	}
//set odometryflag
	void ImageProcesser::set_odomrcvd(void){
		ODOMETRY_RECEIVED=true;
	}
//odometry dx's sign change
	void ImageProcesser::dxsignchange(void){
		global_dx=(-global_dx);
	}
//進行の向きを取得
	bool ImageProcesser::pose_detection(double position_x,double position_y,double prev_yaw){
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

