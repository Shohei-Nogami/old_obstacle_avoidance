#include"estimate_velocity.h"

estimate_velocity::estimate_velocity()
{
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/objects_info",1,&estimate_velocity::objects_callback,this);

  pub_pcl = nh_pcl.advertise<sensor_msgs::PointCloud2>("clusted_cloud2", 1);
	
	pub2 = nh_pub2.advertise<obst_avoid::filted_objects_info>("filted_objects_info", 1);
	//calmanfilter parameter
	
	//Eigen::MatrixXd sig_ut(6,6);
	/*
	sig_ut = Eigen::MatrixXd::Zero(3,3); 
	del_t = Eigen::MatrixXd::Zero(6,6); 
	sig_x0 = Eigen::MatrixXd::Zero(6,6); 
	I = Eigen::MatrixXd::Identity(6,6);
	
	//仮
	sig_ut(0,0)=0.5;
	sig_ut(1,1)=0.5;
	sig_ut(2,2)=0.5;
	
	del_t(0,0)=0.5;
	del_t(1,1)=0;
	del_t(2,2)=0.5;
	del_t(3,3)=0.5;
	del_t(4,4)=0;
	del_t(5,5)=0.5;
	
	sig_x0(0,0)=1;
	sig_x0(1,1)=1;
	sig_x0(2,2)=1;
	sig_x0(3,3)=1;
	sig_x0(4,4)=1;
	sig_x0(5,5)=1;
	*/
	sig_ut = Eigen::MatrixXd::Zero(2,2); 
	del_t = Eigen::MatrixXd::Zero(4,4); 
	sig_x0 = Eigen::MatrixXd::Zero(4,4); 
	sig_wk = Eigen::MatrixXd::Zero(4,4);
	I = Eigen::MatrixXd::Identity(4,4);
	
	//仮
	
	del_t(0,0)=0.05*0.05;//0.01;//x
	del_t(1,1)=0.05*0.05;//0.01;//z
	del_t(2,2)=0.02;//0.09;//0.04;//0.25;//vx
	del_t(3,3)=0.02;//0.09;//0.04;//0.25;//vz
	

	sig_x0(0,0)=0.01;//0.04;//x
	sig_x0(1,1)=0.01;//0.04;//z
	sig_x0(2,2)=0.09;//vx
	sig_x0(3,3)=0.09;//vz
	
	sig_x0(0,2)=sig_x0(2,2)*(0.2*0.2);
	sig_x0(2,0)=sig_x0(0,2);//sig_x0(0,0)/(0.2*0.2);
	sig_x0(1,3)=sig_x0(3,3)*(0.2*0.2);
	sig_x0(3,1)=sig_x0(1,3);//sig_x0(1,1)/(0.2*0.2);
	

	sig_wk(0,0)=0.05*0.05;//x
	sig_wk(1,1)=0.05*0.05;//z
	sig_wk(2,2)=0.02;//0.01;//vx
	sig_wk(3,3)=0.02;//0.01;//vz

	//現在日時を取得する
	time_t t = time(nullptr);
	 
	//形式を変換する    
	const tm* lt = localtime(&t);
	 
	//sに独自フォーマットになるように連結していく
	std::stringstream s;
	s<<"20";
	s<<lt->tm_year-100; //100を引くことで20xxのxxの部分になる
	s<<"-";
	s<<lt->tm_mon+1; //月を0からカウントしているため
	s<<"-";
	s<<lt->tm_mday; //そのまま
	s<<"_";
	s<<lt->tm_hour;
	s<<":";
	s<<lt->tm_min;
	s<<":";
	s<<lt->tm_sec;
	//result = "2015-5-19" 
	std::string result = s.str();
	

	ofilename="obstacle_odom_and_vel"+result+".csv";

}

estimate_velocity::~estimate_velocity()
{
	
}

void estimate_velocity::subscribe_objects(void)
{
	queue.callOne(ros::WallDuration(1));

}


void estimate_velocity::objects_callback(const obst_avoid::objects_info::ConstPtr& msg)
{
	std::cout<<"aa\n";
	if(pre_objs.obj.size())
	{
		
		prepre_objs.obj.clear();
		prepre_objs.obj.resize(pre_objs.obj.size());
		
		
		for(int i=0;i<prepre_objs.obj.size();i++)
		{
			prepre_objs.obj[i].pos.x=pre_objs.obj[i].pos.x;
			prepre_objs.obj[i].pos.y=pre_objs.obj[i].pos.y;
			prepre_objs.obj[i].pos.z=pre_objs.obj[i].pos.z;
			prepre_objs.obj[i].match=pre_objs.obj[i].match;
			prepre_objs.obj[i].size=pre_objs.obj[i].size;
			prepre_objs.obj[i].r=pre_objs.obj[i].r;
			prepre_objs.obj[i].vel=pre_objs.obj[i].vel;
			
			prepre_objs.obj[i].pt.resize(pre_objs.obj[i].pt.size());
			for (int k = 0; k < prepre_objs.obj[i].pt.size(); k++)
			{
				prepre_objs.obj[i].pt[k] = pre_objs.obj[i].pt[k];
			}
		}
		
		//prepre_objs.obj=pre_objs.obj;
		prepre_objs.dt=pre_objs.dt;
		prepre_objs.dX=pre_objs.dX;
		
	}
	std::cout<<"	if(pre_objs.obj.size())\n";
	if(cur_objs.obj.size())
	{	
		pre_objs.obj.clear();
		pre_objs.obj.resize(cur_objs.obj.size());
		
		for(int i=0;i<pre_objs.obj.size();i++)
		{
			pre_objs.obj[i].pos.x=cur_objs.obj[i].pos.x;
			pre_objs.obj[i].pos.y=cur_objs.obj[i].pos.y;
			pre_objs.obj[i].pos.z=cur_objs.obj[i].pos.z;
			pre_objs.obj[i].match=cur_objs.obj[i].match;
			pre_objs.obj[i].size=cur_objs.obj[i].size;
			pre_objs.obj[i].r=cur_objs.obj[i].r;
			pre_objs.obj[i].vel=cur_objs.obj[i].vel;

			pre_objs.obj[i].pt.resize(cur_objs.obj[i].pt.size());
			for (int k = 0; k < pre_objs.obj[i].pt.size(); k++)
			{
				pre_objs.obj[i].pt[k] = cur_objs.obj[i].pt[k];
			}
		}
		
		//pre_objs.obj=cur_objs.obj;
		
		pre_objs.dt=cur_objs.dt;
		pre_objs.dX=cur_objs.dX;
		//std::cout<<"pre,cur:"<<pre_objs.dX<<","<<cur_objs.dX<<"\n";
	}
	std::cout<<"cur_objs)\n";
	cur_objs.obj.clear();
	cur_objs.obj.resize(msg->obj.size());
	
	for(int i=0;i<msg->obj.size();i++)
	{
		cur_objs.obj[i].pos.x=msg->obj[i].pos.x;
		cur_objs.obj[i].pos.y=msg->obj[i].pos.y;
		cur_objs.obj[i].pos.z=msg->obj[i].pos.z;
		cur_objs.obj[i].match=msg->obj[i].match;
		//std::cout<<"cur,msg:"<<cur_objs.obj[i].match<<","<<msg->obj[i].match<<"\n";
		cur_objs.obj[i].size=msg->obj[i].size;
		cur_objs.obj[i].r=msg->obj[i].r;
		cur_objs.obj[i].vel=msg->obj[i].vel;

		cur_objs.obj[i].pt.resize(msg->obj[i].pt.size());
		for (int k = 0; k < msg->obj[i].pt.size(); k++)
		{
			cur_objs.obj[i].pt[k] = msg->obj[i].pt[k];
		}
		//std::cout<<"msg->obj[i].pt.size():"<<msg->obj[i].pt.size()<<"\n";
	}
	
	//cur_objs.obj=msg->obj;
	cur_objs.dt=msg->dt;
	cur_objs.dX=msg->dX;
	//std::cout<<"cur,msg:"<<cur_objs.dX<<","<<msg->dX<<"\n";
	//std::cout<<"aaa\n";
}
bool estimate_velocity::culculate_velocity(void)
{
	std::cout<<"culculate_velocity\n";

	//update vel data
	prepre_vel.resize(pre_vel.size());
	
	for(int i=0;i<pre_vel.size();i++)
	{
		prepre_vel[i]=pre_vel[i];
		//std::cout<<"prepre,pre:"<<prepre_vel[i]<<","<<pre_vel[i]<<"\n";
	}
	
	pre_vel.resize(vel.size());
	for(int i=0;i<vel.size();i++)
	{
		pre_vel[i]=vel[i];
	}
	
	vel.clear();
	vel.resize(cur_objs.obj.size());
	for(int i=0;i<vel.size();i++)
	{
		vel[i].x=0;
		vel[i].y=0;
		vel[i].z=0;
	}
	vel_h.clear();
	vel_h.resize(cur_objs.obj.size());
	for(int i=0;i<vel_h.size();i++)
	{
		vel_h[i].x=0;
		vel_h[i].y=0;
		vel_h[i].z=0;
	}
	
	//uptate acc data
	pre_acc.resize(acc.size());
	for(int i=0;i<acc.size();i++)
	{
		pre_acc[i]=acc[i];
	}
	
	
	
	acc.clear();
	acc.resize(cur_objs.obj.size());
	
	for(int i=0;i<acc.size();i++)
	{
		acc[i].x=0;
		acc[i].y=0;
		acc[i].z=0;
	}
	
	//update tracking data
	if(!prepre_objs.obj.size())
	{
		return false;
	}
	
	if(track_n.size())
	{
		pre_track_n.resize(track_n.size());
		for(int i=0;i<pre_track_n.size();i++)
		{
		  pre_track_n[i]=track_n[i];
		}
	}
	if(!pre_track_n.size())
	{
		pre_track_n.resize(pre_objs.obj.size());
		for(int i=0;i<pre_track_n.size();i++)
		{
		  pre_track_n[i]=0;
		}
	}
	track_n.resize(cur_objs.obj.size());
	for(int i=0;i<track_n.size();i++)
	{
		track_n[i]=0;
	}
	std::cout<<"velocity\n";
	//velocity
	cv::Point3f temp0=cv::Point3f(0,0,0);
	//cluster match[i]=j : i -> j
	for(int i=0;i<vel.size();i++)
	{
		if(cur_objs.obj[i].match ==-1||cur_objs.obj[i].size>0.7*0.7||cur_objs.obj[i].size<0.1*0.1)
		{
			
			continue;
		}
		//if(std::abs( cur_objs.obj[i].match )>  (int)pre_objs.obj.size() )
		if(cur_objs.obj[i].match != -1)
		{
			double T=0.05;			
			vel[i].x = (cur_objs.obj[i].pos.x-pre_objs.obj[ cur_objs.obj[i].match ].pos.x)/cur_objs.dt-cur_objs.dX.x/cur_objs.dt;
			vel[i].y = 0;//(cur_objs.obj[i].pos.y-pre_objs.obj[ cur_objs.obj[i].match ].pos.y)/cur_objs.dt-cur_cluster.dX.y/cur_objs.dt;
			vel[i].z = (cur_objs.obj[i].pos.z-pre_objs.obj[ cur_objs.obj[i].match ].pos.z)/cur_objs.dt-cur_objs.dX.z/cur_objs.dt;
			
			vel[i].x=(vel[i].x*cur_objs.dt+pre_vel[ cur_objs.obj[i].match ].x*T)/(T+cur_objs.dt);
			vel[i].y=0;//(vel[i].y*cur_objs.dt+pre_vel[match_num].y*T)/(T+cur_objs.dt);
			vel[i].z=(vel[i].z*cur_objs.dt+pre_vel[ cur_objs.obj[i].match ].z*T)/(T+cur_objs.dt);
			track_n[i]=pre_track_n[cur_objs.obj[i].match]+1;
			
			///--test code			
			vel[i].x=cur_objs.obj[i].vel.x;
			vel[i].y=0;//cur_objs.obj[i].vel.y;
			vel[i].z=cur_objs.obj[i].vel.z;
			///-------------

			//vel[i].x=(vel[i].x*cur_objs.dt+pre_vel[ cur_objs.obj[i].match ].x*T)/(T+cur_objs.dt);
			//vel[i].y=0;//(vel[i].y*cur_objs.dt+pre_vel[match_num].y*T)/(T+cur_objs.dt);
			//vel[i].z=(vel[i].z*cur_objs.dt+pre_vel[ cur_objs.obj[i].match ].z*T)/(T+cur_objs.dt);
			
			
			//threshold filter
			float dX;
			culc_distance_3f(vel[i],temp0,dX);
			
			if(dX>1.1)
			{
				culc_distance_3f(pre_vel[cur_objs.obj[i].match],temp0,dX);
				if(dX<1.1)
				{
					vel[i] = pre_vel[cur_objs.obj[i].match];
				}
				else
				{
					track_n[i]=0;
					vel[i].x = 0;
					vel[i].y = 0;
					vel[i].z = 0;
				}
			}
			
			
		}
		else
		{
			track_n[i]=0;
			vel[i].x = 0;
			vel[i].y = 0;
			vel[i].z = 0;
		}
	
	}
	return true;
}
void estimate_velocity::culculate_accelerate(void)
{
	std::cout<<"accelerate\n";
	//accelerate

	//cluster match[i]=j : i -> j
	for(int i=0;i<acc.size();i++)
	{
		if(cur_objs.obj[i].size>0.7*0.7||cur_objs.obj[i].size<0.1*0.1)
		{
			continue;
		}
		//if(std::abs( cur_objs.obj[i].match ) <  (int)pre_objs.obj.size() )
		if(cur_objs.obj[i].match!=-1)
		{

			double T=0.1;			
			acc[i].x = (vel[i].x-pre_vel[ cur_objs.obj[i].match ].x)/cur_objs.dt;
			acc[i].y = 0;//(vel[i].y-pre_vel[ cur_objs.obj[i].match ].y)/cur_objs.dt-cur_cluster.dX.y/cur_objs.dt;
			acc[i].z = (vel[i].z-pre_vel[ cur_objs.obj[i].match ].z)/cur_objs.dt;
		
			acc[i].x=(acc[i].x*cur_objs.dt+pre_acc[ cur_objs.obj[i].match ].x*T)/(T+cur_objs.dt);
			acc[i].y=0;//(acc[i].y*cur_objs.dt+pre_acc[match_num].y*T)/(T+cur_objs.dt);
			acc[i].z=(acc[i].z*cur_objs.dt+pre_acc[ cur_objs.obj[i].match ].z*T)/(T+cur_objs.dt);
			
			float dis;
			cv::Point3f temp=cv::Point3f(0,0,0);
			culc_distance_3f(acc[i],temp,dis);

			if(dis>1.1)
			{
				culc_distance_3f(pre_acc[cur_objs.obj[i].match],temp,dis);
				if(dis>1.1)
				{
					acc[i]=temp;
				}
				else
				{
					acc[i]=pre_acc[cur_objs.obj[i].match];
				}
			}
		}
		else
		{
			acc[i].x = 0;
			acc[i].y = 0;
			acc[i].z = 0;
		}
		std::cout<<"vel,acc["<<i<<"]:"<<vel[i]<<","<<acc[i]<<")\n";
	}
}


void estimate_velocity::record_odom_and_vel(void)
{
	

	//std::cout<<"write file:"<<result<< "\n";
	//std::ofstream ofss("./Documents/obstacle_odom_and_vel"+result+".csv",std::ios::app);
	std::ofstream ofss(ofilename,std::ios::app);
	//std::cout<<"cur_objs.obj.size():"<<cur_objs.obj.size()<<"\n";
	if(!ofss)
	{
		int a;
		std::cout<<"	if(!ofss)\n";
		std::cin>>a;
	}
	for(int i=0;i<cur_objs.obj.size();i++)
	{
		if(track_n[i]<1||std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>1.1
			||cur_objs.obj[i].size>1.0*1.0
			||cur_objs.obj[i].size<0.2*0.2
		)
		{
			continue;
		}
		if(vel.size())
		{
			if(acc.size())
			{
				ofss<< cur_objs.obj[i].pos.x <<","//x
					<< cur_objs.obj[i].pos.y <<","//y
					<< cur_objs.obj[i].pos.z <<","//z
					<<"-"<<","
					<<vel[i].x<<","//vx
					<<vel[i].y<<","//vy
					<<vel[i].z<<","//vz
					<<"-"<<","
					<<acc[i].x<<","//ax
					<<acc[i].y<<","//ay
					<<acc[i].z<<","//az
					<<std::endl;

			}
			else
			{
				ofss<< cur_objs.obj[i].pos.x <<","//x
					<< cur_objs.obj[i].pos.y <<","//y
					<< cur_objs.obj[i].pos.z <<","//z
					<<"-"<<","
					<<vel[i].x<<","//vx
					<<vel[i].y<<","//vy
					<<vel[i].z<<","//vz
					<<std::endl;
			}
		}
		else
		{
			ofss<< cur_objs.obj[i].pos.x <<","//x
				<< cur_objs.obj[i].pos.y <<","//y
				<< cur_objs.obj[i].pos.z <<","//z
				<<std::endl;

		}
	}	
	
}

void estimate_velocity::calmanfilter(void)
{
	
	std::cout<<"calmanfilter\n";
	//std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > xh_t;
	//std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > sig_xh_t;
	
	//std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > xh_t_1;
	//std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > sig_xh_t_1;
	
	
	//static
	//Eigen::MatrixXd xh_t(6,1);
	//Eigen::MatrixXd sig_xh_t(6,6);
	/*
	Eigen::MatrixXd d_t(6,6);
	Eigen::MatrixXd K_t(6,6);
	
	
	//public
	Eigen::MatrixXd xt_t(6,1);
	//Eigen::MatrixXd xh_t_1(6,1);
	Eigen::MatrixXd z_t(6,1);
	Eigen::MatrixXd F_t(6,6);
	Eigen::MatrixXd B_t(6,3);
	//Eigen::MatrixXd sig_xh_t_1(6,6);
	Eigen::MatrixXd sig_xt(6,6);
	Eigen::MatrixXd u_t(3,1);
	*/
	Eigen::MatrixXd d_t(4,4);
	Eigen::MatrixXd K_t(4,4);
	
	
	//public
	Eigen::MatrixXd xt_t(4,1);
	//Eigen::MatrixXd xh_t_1(6,1);
	Eigen::MatrixXd z_t(4,1);
	Eigen::MatrixXd F_t = Eigen::MatrixXd::Zero(4,4);
	Eigen::MatrixXd B_t = Eigen::MatrixXd::Zero(2,2);//(4,2);
	//Eigen::MatrixXd sig_xh_t_1(6,6);
	Eigen::MatrixXd sig_xt(4,4);
	Eigen::MatrixXd u_t(4,1);
	

	float dt = cur_objs.dt;
	//set F
	F_t(0,0)=1;
	F_t(1,1)=1;
	F_t(2,2)=1;
	F_t(3,3)=1;
	
	F_t(0,2)=dt;
	F_t(1,3)=dt;

	//x : x,y,z,vx,vy,vz
	//u : ax,ay,az

	//set R,Q
	
	sig_wk(0,2)=sig_wk(2,2)*(dt*dt);
	sig_wk(1,3)=sig_wk(3,3)*(dt*dt);
	sig_wk(2,0)=sig_wk(0,2);//sig_wk(0,0)/(dt*dt);
	sig_wk(3,1)=sig_wk(1,3);//sig_wk(1,1)/(dt*dt);

	//set Q
	del_t(0,2)=del_t(2,2)*(dt*dt);
	del_t(1,3)=del_t(3,3)*(dt*dt);
	del_t(2,0)=del_t(0,2);//del_t(0,0)/(dt*dt);
	del_t(3,1)=del_t(1,3);//del_t(1,1)/(dt*dt);
	
	xh_t_1.resize(xh_t.size());
	sig_xh_t_1.resize(sig_xh_t.size());
	xh_t_1.resize(cur_objs.obj.size());
	sig_xh_t_1.resize(cur_objs.obj.size());
	//std::cout<<"xh_t_1.size:"<<xh_t_1.size()<<"\n";
	//std::cout<<"sig_xh_t_1.size:"<<sig_xh_t_1.size()<<"\n";
	std::cout<<"for init\n";
	std::vector<bool> skip;
	skip.resize(cur_objs.obj.size());
	for(int i=0;i<cur_objs.obj.size();i++)
	{
		skip[i]=false;
		//std::cout<<"in for \n";
		//observation
		/*
		z_t(0,0)=cur_objs.obj[i].pos.x;
		z_t(1,0)=cur_objs.obj[i].pos.y;
		z_t(2,0)=cur_objs.obj[i].pos.z;
		z_t(3,0)=vel[i].x;
		z_t(4,0)=vel[i].y;
		z_t(5,0)=vel[i].z;
		*/
		z_t(0,0)=cur_objs.obj[i].pos.x;
		z_t(1,0)=cur_objs.obj[i].pos.z;
		z_t(2,0)=vel[i].x;
		z_t(3,0)=vel[i].z;
		//std::cout<<"observation \n";
		
		//can't do filter
		if(track_n[i]<2)//1:no tracking,2:vel,3:acc
		{
			xh_t_1[i]=z_t;
			
			sig_xh_t_1[i]=sig_x0;
			
			continue;
		}
		//t -> t-1
		// i の値を cur,pre 間で揃える
		//std::cout<<"i:"<<i<<"\n";
		//std::cout<<"cur_objs.obj["<<i<<"].match:"<<cur_objs.obj[i].match<<"\n";
		if(std::abs( cur_objs.obj[i].match ) >  (int)pre_objs.obj.size() ||cur_objs.obj[i].match ==-1	|| cur_objs.obj[i].size>0.7*0.7||cur_objs.obj[i].size<0.1*0.1)

		//if(cur_objs.obj[i].match !=-1)
		{
			//xh_t_1[i]=z_t;
			
			//sig_xh_t_1[i]=sig_x0;
			skip[i]=true;
			
		}
		else
		{
			xh_t_1[i]=xh_t[ cur_objs.obj[i].match ];
			sig_xh_t_1[i]=sig_xh_t[ cur_objs.obj[i].match ];
		}
	}
	
	xh_t.clear();
	sig_xh_t.clear();
	xh_t.resize(cur_objs.obj.size());
	sig_xh_t.resize(cur_objs.obj.size());
	
	
	std::cout<<"for filter\n";
	for(int i=0;i<cur_objs.obj.size();i++)
	{

		//set z
		z_t(0,0)=cur_objs.obj[i].pos.x;
		z_t(1,0)=cur_objs.obj[i].pos.z;
		z_t(2,0)=vel[i].x;
		z_t(3,0)=vel[i].z;

		//can't do filter
		if(track_n[i]<2)//||cur_objs.obj[i].pos.x==xh_t_1[i](0,0))//z_t==xh_t_1[i](0,0) -> skip
		{	
			xh_t[i]=z_t;
			sig_xh_t[i]=sig_x0;
			continue;
		}
		if(skip[i])
		{
			xh_t[i]=z_t;
			sig_xh_t[i]=sig_x0;
			continue;
		}



		//filtering
		
		//std::cout<<"filtering\n";
		float dt = cur_objs.dt;
		
		//set ut
		
		//u_t(0,0)=acc[i].x;
		//u_t(1,0)=acc[i].z;
		u_t(0,0)=pre_vel[ cur_objs.obj[i].match ].x;//vel[i].x;
		u_t(1,0)=pre_vel[ cur_objs.obj[i].match ].z;//vel[i].z;
		
		//std::cout<<"set ut\n";
		//predict

		//std::cout<<"F_t:"<<F_t<<"\n";
		//std::cout<<"B_t:"<<B_t<<"\n";
		//std::cout<<"sig_xh_t_1["<<i<<"]:"<<sig_xh_t_1[i]<<"\n";
		//std::cout<<"sig_ut:"<<sig_ut<<"\n";
		//std::cout<<"xh_t_1["<<i<<"]:"<<xh_t_1[i]<<"\n";
		//std::cout<<"u_t:"<<u_t<<"\n";
		
		xt_t = F_t*xh_t_1[i];// + B_t*u_t ;
		sig_xt = F_t * sig_xh_t_1[i] * F_t.transpose()+sig_wk;// + B_t * sig_ut * B_t.transpose() ;
		
		
		K_t = sig_xt*( (sig_xt+del_t).inverse() );
		xh_t[i] = xt_t + K_t*( z_t - xt_t );
		sig_xh_t[i] = (I - K_t)*sig_xt;


		vel_h[i].x=xh_t[i](2,0);
		vel_h[i].z = xh_t[i](3,0);
		vel_h[i].y=0;
		std::cout<<"vel["<<i<<"]:"<<vel[i]<<"\n";
		std::cout<<"vel_h["<<i<<"]:"<<vel_h[i]<<"\n";
		std::cout<<"xh_t[i]:"<<xh_t[i]<<"\n";
		std::cout<<" cur_objs.obj[i].pos:"<< cur_objs.obj[i].pos<<"\n";
		std::cout<<" sig_xh_t[i]:"<< sig_xh_t[i] <<"\n";
	}
}


void estimate_velocity::culc_distance_3f(const cv::Point3f& x1,cv::Point3f& x2,float& dis)
{
	dis = std::sqrt((x1.x-x2.x)*(x1.x-x2.x) + (x1.y-x2.y)*(x1.y-x2.y) + (x1.z-x2.z)*(x1.z-x2.z) );
}
void estimate_velocity::LPF(float& x_t,const float& x_t_1,double& dt,const double& T)
{
	x_t=(x_t*dt+x_t_1*T)/(T+dt);
} 


void estimate_velocity::publish_pointcloud(void)
{
	int j = 0;
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	clusted_cloud->points.clear();
	clusted_cloud->points.reserve(673*376);

	pcl::PointXYZRGB cloud_temp;
	float resolution=0.05;
	for(int i=0;i<cur_objs.obj.size();i++)
	{
		///std::cout<<"i:"<<i<<"\n";
		//std::cout<<"track_n[i]:"<<track_n[i]<<"\n";
		//std::cout<<"std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0)):"<<std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))<<"\n";
		//std::cout<<"cur_objs.obj[i].size:"<<cur_objs.obj[i].size<<"\n";
		//std::cout<<"(cur_objs.obj[i].r:"<<cur_objs.obj[i].r<<"\n";
						
		for(int t=0;t<4;t++)
		{
			
			if(track_n[i]<1||std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>1.1
				||cur_objs.obj[i].size>0.7*0.7
				||cur_objs.obj[i].size<0.2*0.2
			)
			{
				continue;
			}
			if(t>0)
			{
				if(std::sqrt(sig_xh_t[i](2, 2) + sig_xh_t[i](3, 3) ) 
					//> std::sqrt(std::pow(vel_h[i].x, 2.0) + std::pow(vel_h[i].z, 2.0))
					> std::sqrt(std::pow(vel[i].x, 2.0) + std::pow(vel[i].z, 2.0))&&
					std::sqrt(std::pow(vel[i].x, 2.0) + std::pow(vel[i].z, 2.0))>0.2
					)
					{
						continue;
					}
			}
/*
			for(int w=-(int)(cur_objs.obj[i].r/resolution);w<=(int)(cur_objs.obj[i].r/resolution);w++)
			{
				for(int d=-(int)(cur_objs.obj[i].r/resolution);d<=(int)(cur_objs.obj[i].r/resolution);d++)
				{
					//int d=0;
					
					cloud_temp.y=-cur_objs.obj[i].pos.x+w*resolution;
					cloud_temp.z=cur_objs.obj[i].pos.y+0.4125;//+d*resolution;
					cloud_temp.x=cur_objs.obj[i].pos.z+d*resolution;
					cloud_temp.r=colors[i%12][0];
					cloud_temp.g=colors[i%12][1];
					cloud_temp.b=colors[i%12][2];
					cloud_temp.x+=vel[i].z*t;
				  cloud_temp.y+=-vel[i].x*t;			
				  cloud_temp.z+=vel[i].y*t;
				
				  cloud_temp.y+=10;		

					clusted_cloud->points.push_back(cloud_temp);
				}
			}		
	*/
			for(int k=0;k<cur_objs.obj[i].pt.size();k++)
			{
				if(1)//k%30)
				{
					continue;
				}
				//cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
				//cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
				//cloud_temp.y=-(cur_objs.obj[i].pt[k].x*ksize-width/2)*cur_objs.obj[i].pt[k].z/f;
				//cloud_temp.z=((height/2-cur_objs.obj[i].pt[k].y*ksize)*cur_objs.obj[i].pt[k].z)/f+0.4125;
				cloud_temp.y=-(cur_objs.obj[i].pt[k].x-width/2)*cur_objs.obj[i].pt[k].z/f;
				cloud_temp.z=((height/2-cur_objs.obj[i].pt[k].y)*cur_objs.obj[i].pt[k].z)/f+0.4125;
				cloud_temp.x=cur_objs.obj[i].pt[k].z;
				cloud_temp.r=colors[i%12][0];
				cloud_temp.g=colors[i%12][1];
				cloud_temp.b=colors[i%12][2];
				cloud_temp.x+=vel[i].z*t;
		    cloud_temp.y+=-vel[i].x*t;			
		    cloud_temp.z+=vel[i].y*t;
				
		    cloud_temp.y+=10;		

				clusted_cloud->points.push_back(cloud_temp);
			}	
		}
		for(int t=0;t<4;t++)
		{
			if(t==0)
			{
				for(int k=0;k<cur_objs.obj[i].pt.size();k++)
				{
					if(k%10)//30)
					{
						continue;
					}
					//cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
					//cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].pt[k].z)/f+0.4125;

					//cloud_temp.y=-(cur_objs.obj[i].pt[k].x*ksize-width/2)*cur_objs.obj[i].pt[k].z/f;
					//cloud_temp.z=((height/2-cur_objs.obj[i].pt[k].y*ksize)*cur_objs.obj[i].pt[k].z)/f+0.4125;
					cloud_temp.y=-(cur_objs.obj[i].pt[k].x-width/2)*cur_objs.obj[i].pt[k].z/f;
					cloud_temp.z=((height/2-cur_objs.obj[i].pt[k].y)*cur_objs.obj[i].pt[k].z)/f+0.4125;
					cloud_temp.x=cur_objs.obj[i].pt[k].z;
					cloud_temp.r=colors[i%12][0];
					cloud_temp.g=colors[i%12][1];
					cloud_temp.b=colors[i%12][2];
				
					cloud_temp.y+=10;		
					cloud_temp.z+=4;
					clusted_cloud->points.push_back(cloud_temp);
				}	
				continue;
			}
			
			if(track_n[i]<1//||std::sqrt(std::pow(vel[i].x,2.0)+std::pow(vel[i].z,2.0))>1.1
				||std::sqrt(std::pow(vel_h[i].x,2.0)+std::pow(vel_h[i].z,2.0))>1.1
					//std::sqrt(std::pow(xh_t[i](2, 0),2.0)+std::pow(xh_t[i](3, 0),2.0))>1.1
					||cur_objs.obj[i].size>0.7*0.7
					||cur_objs.obj[i].size<0.2*0.2
			)
			{
				continue;
			}
			if (t > 0) {
				if (
					
					
						//std::sqrt(std::pow(sig_xh_t[i](2, 2), 2.0) + std::pow(sig_xh_t[i](2, 0), 2.0)) < xh_t[i](2, 0)
					//||std::sqrt(std::pow(sig_xh_t[i](3, 3), 2.0) + std::pow(sig_xh_t[i](3, 1), 2.0)) < xh_t[i](3, 0)
					/*
					std::sqrt(sig_xh_t[i](2, 2) + sig_xh_t[i](2, 0) +
						sig_xh_t[i](3, 3) + sig_xh_t[i](3, 1) ) 
					> std::sqrt(std::pow(xh_t[i](2, 0), 2.0) + std::pow(xh_t[i](3, 0), 2.0))/2
					*/
					
					//std::sqrt(sig_xh_t[i](2, 2) + sig_xh_t[i](3, 3) ) 
					std::sqrt(sig_xh_t[i](2, 0)+sig_xh_t[i](2, 2) + sig_xh_t[i](3, 1)+sig_xh_t[i](3, 3) ) 
					> std::sqrt(std::pow(vel_h[i].x, 2.0) + std::pow(vel_h[i].z, 2.0))
					//||
					//std::sqrt(sig_xh_t[i](0, 0) + sig_xh_t[i](0, 2) +sig_xh_t[i](1, 1) + sig_xh_t[i](1, 3) ) 
					//> std::sqrt(std::pow(vel_h[i].x, 2.0) + std::pow(vel_h[i].z, 2.0))/2
					
					//> std::sqrt(std::pow(vel[i].x, 2.0) + std::pow(vel[i].z, 2.0))
					//|| std::sqrt(std::pow(xh_t[i](2, 0), 2.0) + std::pow(xh_t[i](3, 0), 2.0)) < 0.1
					//||track_n[i]<10
					)
				{
					continue;
				}
			}
			/*
			for(int w=-(int)(cur_objs.obj[i].r/resolution);w<=(int)(cur_objs.obj[i].r/resolution);w++)
			{
				for(int d=-(int)(cur_objs.obj[i].r/resolution);d<=(int)(cur_objs.obj[i].r/resolution);d++)
				{
					//int d=0;
					
					cloud_temp.y = -xh_t[i](0, 0) + w * resolution;
					cloud_temp.z= 0 +0.4125;//+d*resolution;
					cloud_temp.x = xh_t[i](1, 0) + d * resolution;
					cloud_temp.r = colors[i % 12][0];
					cloud_temp.g = colors[i % 12][1];
					cloud_temp.b = colors[i % 12][2];
					cloud_temp.x += xh_t[i](3, 0)*t;
					cloud_temp.y += -xh_t[i](2, 0)*t;
				  cloud_temp.z += 0*t;
				
				  cloud_temp.y+=10;		
					cloud_temp.z+=4;
					clusted_cloud->points.push_back(cloud_temp);
				}
			}		
			*/
			for(int k=0;k<cur_objs.obj[i].pt.size();k++)
			{
				if(k%10)//30)
				{
					continue;
				}
				//cloud_temp.y=-(cur_cluster.clst[i].pt[k].x*ksize-width/2)*cur_cluster.clst[i].pt[k].z/f;
				//cloud_temp.z=((height/2-cur_cluster.clst[i].pt[k].y*ksize)*cur_cluster.clst[i].pt[k].z)/f+0.4125;
				cv::Point3f temp;
				temp.x=xh_t[i](0, 0)-cur_objs.obj[i].pos.x;
				temp.z=xh_t[i](1, 0)-cur_objs.obj[i].pos.z;

				//cloud_temp.y=-(cur_objs.obj[i].pt[k].x*ksize-width/2)*cur_objs.obj[i].pt[k].z/f;
				//cloud_temp.z=((height/2-cur_objs.obj[i].pt[k].y*ksize)*cur_objs.obj[i].pt[k].z)/f+0.4125;
				cloud_temp.y=-(cur_objs.obj[i].pt[k].x-width/2)*cur_objs.obj[i].pt[k].z/f;
				cloud_temp.z=((height/2-cur_objs.obj[i].pt[k].y)*cur_objs.obj[i].pt[k].z)/f+0.4125;
				cloud_temp.x=cur_objs.obj[i].pt[k].z;
				cloud_temp.r=colors[i%12][0];
				cloud_temp.g=colors[i%12][1];
				cloud_temp.b=colors[i%12][2];
				cloud_temp.x+=vel_h[i].z*t+temp.z;
		    cloud_temp.y+=-vel_h[i].x*t-temp.x;	
		    cloud_temp.z+=0;
				
			  cloud_temp.y+=10;		
				cloud_temp.z+=4;
				clusted_cloud->points.push_back(cloud_temp);
			}	
		}
	}
	std::cout<<"clusted_cloud->points.size():"<<clusted_cloud->points.size()<<"\n";
	clusted_cloud->height=1;
	clusted_cloud->width=clusted_cloud->points.size();
	std::cout<<"cloud->points.size():"<<clusted_cloud->points.size()<<"\n";
	if(!clusted_cloud->points.size())
	{
		return ;
	}
	sensor_msgs::PointCloud2 edit_cloud;
	pcl::toROSMsg (*clusted_cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
	pub_pcl.publish(edit_cloud);
}

void estimate_velocity::publish_filted_objects_info(void)
{
	obst_avoid::filted_objects_info obj_info;

	obj_info.objs.resize(cur_objs.obj.size());

	for (int i = 0; i < obj_info.objs.size(); i++)
	{
		obj_info.objs[i].pos.x = cur_objs.obj[i].pos.x;
		obj_info.objs[i].pos.y = cur_objs.obj[i].pos.y;
		obj_info.objs[i].pos.z = cur_objs.obj[i].pos.z;
		obj_info.objs[i].fpos.x = xh_t[i](0, 0);
		obj_info.objs[i].fpos.y = cur_objs.obj[i].pos.y;
		obj_info.objs[i].fpos.z = xh_t[i](1, 0);
		obj_info.objs[i].r = cur_objs.obj[i].r;
		obj_info.objs[i].size = cur_objs.obj[i].size;
		obj_info.objs[i].vel.x = vel_h[i].x;//xh_t[i](2, 0);
		obj_info.objs[i].vel.y = 0;
		obj_info.objs[i].vel.z = vel_h[i].z;//xh_t[i](3, 0);
		obj_info.objs[i].vdsp.x = sig_xh_t[i](2, 0) + sig_xh_t[i](2, 2);
		obj_info.objs[i].vdsp.y = 0;
		obj_info.objs[i].vdsp.z = sig_xh_t[i](3, 1) + sig_xh_t[i](3, 3);
		
		obj_info.objs[i].pt = cur_objs.obj[i].pt;

	}
	pub2.publish(obj_info);
}


int main(int argc,char **argv)
{
  ros::init(argc,argv,"estimate_velocity_class_test");
 
  estimate_velocity est_vel_cls;
  time_class time_cls;
  while(ros::ok())
	{
    time_cls.set_time();
		std::cout<<"begin\n";
		est_vel_cls.subscribe_objects();
		std::cout<<"subscribed\n";
		if(!est_vel_cls.culculate_velocity())
		{
			continue;
		}
		est_vel_cls.culculate_accelerate();
		std::cout<<"culculate_accelerate\n";
		est_vel_cls.calmanfilter();
		std::cout<<"4:calmanfilter:"<<time_cls.get_time_now()<<"\n";
//		est_vel_cls.record_odom_and_vel();
//		std::cout<<"record_odom_and_vel\n";
		est_vel_cls.publish_pointcloud();
		std::cout<<"4:publish_pointcloud:"<<time_cls.get_time_now()<<"\n";
		est_vel_cls.publish_filted_objects_info();
		std::cout<<"4:publish_filted_objects_info:"<<time_cls.get_time_now()<<"\n";
		std::cout<<"loop\n";
    std::cout<<"dt:"<<time_cls.get_delta_time()<<"\n";
	}
}


