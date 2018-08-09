
#include"avoid_cls.h"

#define STATIC_OBSTACLE 0
#define DYNAMIC_OBSTACLE 1
#define MOVING_OBSTACLE 2

#define SAFETY_OBSTACLE 20
#define WARNING_OBSTACLE 21
#define DANGER_OBSTACLE 22

#define DEFAULT_SPEED 0
#define MIN_SPEED 1
#define MAX_SPEED 2

#define TC_MIN 0
#define TC 1
#define TC_MAX 2

#define ROBOT_STOP -1
#define ROBOT_GO 1

avoid::avoid()
{
	nh_sub.setCallbackQueue(&queue);
	sub = nh_sub.subscribe("filted_objects_info", 1, &avoid::objects_callback, this);

	nh_sub2.setCallbackQueue(&queue2);
	sub2 = nh_sub2.subscribe("robot_odm", 1, &avoid::odometry_callback, this);

	pub2 = nh_pub2.advertise<obst_avoid::select_theta>("select_theta", 1);

	pub_pcl = nh_pub.advertise<sensor_msgs::PointCloud2>("intersection_cloud", 1);
	
	//vfh.vfh_class();


	//set vel temp
	temp_vel.resize(3);
	temp_vel[DEFAULT_SPEED]=0.2;
	temp_vel[MIN_SPEED]=0.1;
	temp_vel[MAX_SPEED]=0.3;

	//check_collision
	t_c_min_angle.resize(max_angle-min_angle);//max_angle-min_angle);//min:-45,max:+45
	for(int i=0;i<t_c_min_angle.size();i++)
	{
		t_c_min_angle[i].resize(3);
	}
	not_select_angle.resize(max_angle-min_angle);

}
avoid::~avoid()
{
	//vfh.~vfh_class();
}
void avoid::set_param_vfh(void)
{
	vfh.set_param(min_angle,max_angle,vfh_resolution,
							robot_r,d_r,mv_length,
							max_vel_dif);
}
void avoid::subscribe_objects(void)
{
	queue.callOne(ros::WallDuration(1));
}

void avoid::objects_callback(const obst_avoid::filted_objects_info::ConstPtr& msg)
{
	obj_info.objs = msg->objs;
}

void avoid::subscribe_odometry(void)
{
	queue2.callOne(ros::WallDuration(1));
}

void avoid::odometry_callback(const obst_avoid::robot_odm::ConstPtr& msg)
{
	robot_odm.x=msg->x;
	robot_odm.y=msg->y;
	robot_odm.th=msg->th;
	x0.x = msg->x;
	x0.y = msg->y;
	theta0 = msg->th;
}

bool avoid::dicriminate_obstacle(void)
{
	if(!obj_info.objs.size())
	{
		return false;
	}
	obstacle_status.resize(obj_info.objs.size());
	obstacle_safety_status.resize(obj_info.objs.size());
	clear_safety_status();
	for (int i = 0; i < obj_info.objs.size(); i++)
	{


		if (obj_info.objs[i].size > 0.7*0.7 || obj_info.objs[i].size < 0.2*0.2
			|| std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) < 0.1
			)
		{
			obstacle_status[i] = STATIC_OBSTACLE;
		}
		else if (std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) > 1.1
			|| std::sqrt(obj_info.objs[i].vdsp.x + obj_info.objs[i].vdsp.z)
				> std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) )
		{
			obstacle_status[i] = DYNAMIC_OBSTACLE;
		}
		else
		{
			obstacle_status[i] = MOVING_OBSTACLE;
		}

	}
	return true;
}

void avoid::set_equation(void)
{
	obj_eq_a.clear();
	obj_eq_a.resize(obj_info.objs.size());

	for (int i = 0; i < obj_eq_a.size(); i++)
	{
		if(obj_info.objs[i].vel.x!=0)
		{
			obj_eq_a[i] = obj_info.objs[i].vel.z / obj_info.objs[i].vel.x;
		}
		else
		{
			obj_eq_a[i]=0;
		}
	}
}
void avoid::set_grid_map(void)
{
	vfh.clear_grid_map();
	const int th_pt_size=10;
	//set grid map
	for (int i = 0; i < obj_info.objs.size(); i++)
	{
		if(th_pt_size>(int)obj_info.objs.size())
		{
			continue;
		}
		if(obstacle_status[i] == MOVING_OBSTACLE)
		{
			continue;
		}
		vfh.set_grid_map(obj_info.objs[i].pt);
	}
}
void avoid::select_route(void)
{

	//select_route
	//cv::Point2f x0 = cv::Point2f(0, 0);
	cv::Point2f xp = cv::Point2f(0, 10);
	//float theta0 = 0;

	std::cout<<"x0,xp:"<<x0<<","<<xp<<"\n";
	std::cout<<"robot_odm:"<<robot_odm<<"\n";

	//修正が必要
	selected_angle_i = vfh.select_best_trajectory(x0, theta0, xp, w_target, w_angle);

	selected_angle=((min_angle + selected_angle_i * (max_angle - min_angle) / (vfh_resolution))*M_PI / 180);
}
//selected_angle：float
//selected_agnle_i:int
//どちらも取得できるように後で改良

//dangerous angleをvfh+に反映できるように改良すべき

void avoid::set_intersection(void)
{
	float a_r;
	float a_x,a_y;
	//float selected_angle=(min_angle+selected_angle_i*(max_angle-min_angle)/vfh_resolution)*M_PI/180;
	if(selected_angle!=0)
	{
		a_r=1.0/tan(selected_angle);
		a_y=selected_vel*std::cos(selected_angle);
		a_x=selected_vel*std::sin(selected_angle);
	}
	else
	{
		a_r=0;
	}
	x_c.clear();
	x_c.resize(obj_info.objs.size());
	float selected_vel_x=selected_vel*sin(selected_angle);
  float selected_vel_z=selected_vel*cos(selected_angle);
  std::cout<<"vx,vz:"<<selected_vel_x<<","<<selected_vel_z<<"\n";

	for(int i = 0; i < obj_info.objs.size(); i++)
	{
		if(obstacle_status[i]!=MOVING_OBSTACLE)
		{
			continue;
		}
		if(std::abs( atan2(obj_info.objs[i].vel.x-a_x,obj_info.objs[i].vel.z-a_y))<M_PI/180*5)//+-5度
		{

		  //float selected_vel_x=selected_vel*sin(selected_angle);
		  //float selected_vel_z=selected_vel*cos(selected_angle);
		  //std::cout<<"vx,vz:"<<selected_vel_x<<","<<selected_vel_z<<"\n";
		  float vel_dif=std::sqrt(std::pow(obj_info.objs[i].vel.x+selected_vel_x,2.0)+std::pow(obj_info.objs[i].vel.z+selected_vel_z,2.0));
		  //std::cout<<"vel_dif:"<<vel_dif<<"\n";

		  /*
			while(ros::ok())
			{
				std::cout<<"vel_dif:"<<std::sqrt(std::pow(obj_info.objs[i].vel.x+selected_vel_x,2.0)+std::pow(obj_info.objs[i].vel.z+selected_vel_z,2.0))<<"\n";
				std::cout<<"vel_dif-selected_vel:"<<vel_dif-selected_vel<<"\n";
			}
			*/

		  if(vel_dif-selected_vel>0)
		  {
		  	obstacle_safety_status[i]=SAFETY_OBSTACLE;//NO COLLISSION
		  }
		  else
		  {
		    //COLLISSION
		    obstacle_safety_status[i]=DANGER_OBSTACLE;

		  }
		  continue;
		}
		else if(obj_info.objs[i].vel.x==0)//not && a_r==0  -> std::abs(a_r-obj_eq_a[i])<M_PI/180*5)
		{
			//float obj_vel=std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0));
			x_c[i].x=obj_info.objs[i].pos.x;
			x_c[i].y=a_r*obj_info.objs[i].pos.x;

		}
		else if(a_x==0)
		{
			x_c[i].x=0;
			x_c[i].y=(0-obj_info.objs[i].pos.x)*obj_eq_a[i]+obj_info.objs[i].pos.z;

		}
		else
		{
			x_c[i].x=(-obj_eq_a[i]*obj_info.objs[i].pos.x+obj_info.objs[i].pos.z)/(a_r+obj_eq_a[i]);
			x_c[i].y=x_c[i].x*a_r;
		}
		std::cout<<"x_c["<<i<<"]:"<<x_c[i]<<"\n";

	}

}
void avoid::set_intersection_time(void)
{
	t_c.clear();
	t_c.resize(obj_info.objs.size());

	for(int i = 0; i < obj_info.objs.size(); i++)
	{
		if(obstacle_status[i]==MOVING_OBSTACLE)
		{
			float length=std::sqrt(std::pow(x_c[i].x,2.0)+std::pow(x_c[i].y,2.0));
			t_c[i]= length/selected_vel;
		}
		else if(obstacle_safety_status[i]==DANGER_OBSTACLE)
		{
			float obstacle_vel=std::sqrt(std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.z,2.0));

		    float length=std::sqrt(std::pow(obj_info.objs[i].pos.x,2.0)+std::pow(obj_info.objs[i].pos.z,2.0));
		    t_c[i] = length/(selected_vel-obstacle_vel);

		}
	}

}
void avoid::clear_safety_status(void)
{
	for(int i = 0; i < obj_info.objs.size(); i++)
	{
    obstacle_safety_status[i]=SAFETY_OBSTACLE;
	}
}
void avoid::labeling_dangerous_obstacle(void)
{

	for(int i = 0; i < obj_info.objs.size(); i++)
	{
		if(obstacle_status[i]==MOVING_OBSTACLE||obstacle_safety_status[i]==DANGER_OBSTACLE)
	  {
			std::cout<<"t_c["<<i<<"]:"<<t_c[i]<<"\n";
			std::cout<<"obj_info.objs["<<i<<"]:"<<obj_info.objs[i].vel<<"\n";
		}
	  if(obstacle_status[i]==MOVING_OBSTACLE)
	  {
	    if(t_c[i]<0||t_c[i]>10)
	    {
	      obstacle_safety_status[i]=SAFETY_OBSTACLE;
	    }
	    else
	    {
	      obstacle_safety_status[i]=WARNING_OBSTACLE;
	    }
	  }
	}
}

void avoid::set_dengerous_time_range(void)
{
  t_c_min.clear();
  t_c_max.clear();
  t_c_min.resize(obj_info.objs.size());
  t_c_max.resize(obj_info.objs.size());

  const double robot_r=0.2;


  for(int i = 0; i < obj_info.objs.size(); i++)
  {
  	if(obstacle_safety_status[i]!=WARNING_OBSTACLE)
  	{
  		continue;
  	}

    double obstacle_r=obj_info.objs[i].r;
		//double obstacle_vel=std::sqrt(std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.z,2.0));
		/*
		double delta_t=std::sqrt( (std::pow(robot_r,2.0)+std::pow(obstacle_r,2.0))
															/(std::pow(selected_vel,2.0)+std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.z,2.0)) );
		*/
		double delta_t=std::sqrt( (std::pow(robot_r,2.0)+std::pow(obstacle_r,2.0))
															/(std::pow(selected_vel,2.0)+std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.z,2.0)) -2*selected_vel*std::sqrt(std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.z,2.0))*selected_vel*sin( atan2(obj_info.objs[i].vel.z,obj_info.objs[i].vel.x)-(-selected_angle) ) );

		t_c_min[i]=t_c[i]-delta_t;
		t_c_max[i]=t_c[i]+delta_t;
		std::cout<<"t_c_min["<<i<<"]:"<<t_c_min[i]<<"\n";
		std::cout<<"t_c_max["<<i<<"]:"<<t_c_max[i]<<"\n";
		std::cout<<"selected_angle,tan:"<<selected_angle<<","<<tan(selected_angle)<<"\n";
  }
}
/*
std::vector<float> temp_vel;
temp_vel.resize(3);
temp_vel[DEFAULT_SPEED]=0.2;
temp_vel[MIN_SPEED]=0.1;
temp_vel[MAX_SPEED]=0.3;
selected_vel_num=DEFAULT_SPEED;

selected_vel=temp_vel[selected_vel_num];
*/
bool avoid::check_collision(void)//any collision -> true; no collision -> false
{


  float t_c_min_all=100;


  float selected_vel_x=selected_vel*sin(selected_angle);
  float selected_vel_z=selected_vel*cos(selected_angle);


  for(int i = 0; i < obj_info.objs.size(); i++)
	{
	  if(obstacle_safety_status[i]!=WARNING_OBSTACLE)
	  {
			if(obstacle_safety_status[i]==DANGER_OBSTACLE)
			{
				if(t_c_min_all>t_c[i])
				{
					t_c_min_all=t_c[i];

				}
			}
	    continue;
	  }

	  cv::Point2f xr,xo;

	  bool COLLISION=true;

	  float collision_t;
	  bool status;
		int PROCESS_TC=TC_MIN;

		switch(PROCESS_TC)
		{
			case TC_MIN:
			{
				//std::cout<<"tc_min\n";
				//tc min
				xr.x=0+selected_vel_x*t_c_min[i];
				xr.y=0+selected_vel_z*t_c_min[i];

				xo.x=obj_info.objs[i].pos.x+obj_info.objs[i].vel.x*t_c_min[i];
				xo.y=obj_info.objs[i].pos.z+obj_info.objs[i].vel.z*t_c_min[i];

				///---hold on
				status=is_collision(xr,xo,obj_info.objs[i].r);
				///-----------
				if(status==COLLISION)
				{
					std::cout<<"t_c_min["<<i<<"]:collision\n";
					if(t_c_min_all>t_c_min[i])
					{
						t_c_min_all=t_c_min[i];
						break;
					}
				}
				PROCESS_TC=TC;
			}
			case TC:
			{
				//std::cout<<"tc\n";
				//tc
				xr.x=0+selected_vel_x*t_c[i];
				xr.y=0+selected_vel_z*t_c[i];

				xo.x=obj_info.objs[i].pos.x+obj_info.objs[i].vel.x*t_c[i];
				xo.y=obj_info.objs[i].pos.z+obj_info.objs[i].vel.z*t_c[i];

				///---hold on
				status=is_collision(xr,xo,obj_info.objs[i].r);
				///-----------
				if(status==COLLISION)
				{
					std::cout<<"t_c:collision\n";
					if(t_c_min_all>t_c[i])
					{
						t_c_min_all=t_c[i];
						break;
					}
					PROCESS_TC=TC_MAX;
				}
			}
			case TC_MAX:
			{
				//std::cout<<"tc_max\n";
				//tc max
				xr.x=0+selected_vel_x*t_c_max[i];
				xr.y=0+selected_vel_z*t_c_max[i];

				xo.x=obj_info.objs[i].pos.x+obj_info.objs[i].vel.x*t_c_max[i];
				xo.y=obj_info.objs[i].pos.z+obj_info.objs[i].vel.z*t_c_max[i];

				///---hold on
				status=is_collision(xr,xo,obj_info.objs[i].r);
				///-----------
				if(status==COLLISION)
				{
					std::cout<<"t_c:collision\n";
					if(t_c_min_all>t_c[i])
					{
						t_c_min_all=t_c[i];
						break;
					}
					PROCESS_TC=TC_MAX;
				}
			}
		}


	  if(status==COLLISION)
	  {
	    obstacle_safety_status[i]=DANGER_OBSTACLE;
	  }
	  else
	  {
	    obstacle_safety_status[i]=SAFETY_OBSTACLE;
	  }
	}
	//std::cout<<"t_c_min_angle.size:"<<t_c_min_angle.size()<<"\n";
	//std::cout<<"selected_angle_i:"<<selected_angle_i<<"\n";
	//std::cout<<"t_c_min_angle[selected_angle_i].size():"<<t_c_min_angle[selected_angle_i].size()<<"\n";
	//std::cout<<"selected_vel_num:"<<selected_vel_num<<"\n";
	if(t_c_min_all>0&&t_c_min_all!=100)
	{
		std::cout<<"return true::t_c_min_all:"<<t_c_min_all<<"\n";
		t_c_min_angle[selected_angle_i][selected_vel_num]=t_c_min_all;
		return true;
	}
	std::cout<<"return true::t_c_min_all:"<<t_c_min_all<<"\n";
	return false;
}
bool avoid::is_collision(cv::Point2f& xr,cv::Point2f& xo,double& objs_r)
{
		float dis=std::sqrt((xr.x-xo.x)*(xr.x-xo.x) + (xr.y-xo.y)*(xr.y-xo.y));
		if(dis<objs_r+robot_r+d_r)
		{
			return true;
		}
		else
		{
			return false;
		}
}

bool avoid::change_selected_vel(void)
{
	selected_vel_num++;
	//std::cout<<"selected_vel_num,(int)temp_vel.size():"<<selected_vel_num<<","<<(int)temp_vel.size()<<"\n";
	if(selected_vel_num>=(int)temp_vel.size())
	{
		return false;
	}
	return true;
}
void avoid::set_selected_vel(void)
{
	selected_vel=temp_vel[selected_vel_num];
}
void avoid::set_default_vel_num(void)
{
	selected_vel_num=DEFAULT_SPEED;
}
bool avoid::select_safety_vel(void)//if safety vel is discovered -> true ,is not -> flase
{
	set_default_vel_num();
	do
	{
		set_selected_vel();
		std::cout<<"selected_vel:"<<selected_vel<<"\n";
		set_intersection();
		set_intersection_time();
		std::cout<<"set_intersection_time\n";
		labeling_dangerous_obstacle();
		std::cout<<"labeling_dangerous_obstacle\n";
		set_dengerous_time_range();
		std::cout<<"set_dengerous_time_range\n";
		if(!check_collision())//if no collision
		{
			return true;
		}
		std::cout<<"selected_vel_num:"<<selected_vel_num<<"\n";
	}while(change_selected_vel());

	return false;
}
/*
std::vector<bool> not_select_angle;

not_select_angle.resize((max_angle-min_angle)/resolution);

in constractor

for(int i=0;i<not_select_angle.size();i++)
{
	not_select_angle[i]=false;
}
*/

bool avoid::set_dangerous_angle(void)
{

	int dangerous_angle=selected_angle_i;
	int angle_range_i=5;//+-5度
	std::cout<<"dangerous_angle:"<<dangerous_angle<<"\n";
	for(int theta_i=dangerous_angle-angle_range_i;theta_i<=dangerous_angle+angle_range_i;theta_i++)
	{
		if(theta_i<0||theta_i>max_angle-min_angle-1)
		{
			continue;
		}
		not_select_angle[theta_i]=true;
	}
	for(int i=0;i<not_select_angle.size();i++)
	{
		if(!not_select_angle[i])//remain select angle
		{
			set_not_select_angle_to_vfh();
			return true;
		}
	}
	return false;
}
void avoid::set_not_select_angle_to_vfh(void)
{
	vfh.set_not_select_angle(not_select_angle);
}
void avoid::clear_not_select_angle(void)
{
	std::cout<<"not_select_angle.size():"<<not_select_angle.size()<<"\n";
	for(int i=0;i<not_select_angle.size();i++)
	{
		not_select_angle[i]=false;
	}
}

void avoid::init_data(void)
{
	clear_not_select_angle();
}

void avoid::set_safety_vel(void)
{
	safety_vel=selected_vel;
	safety_angle=selected_angle;
	std::cout<<"safetyVel:"<<	safety_vel<<","<<safety_angle<<"\n";
}
void avoid::set_stop_vel(void)
{
	safety_vel=0;
	safety_angle=0;
}

void avoid::publish_velocity(void)
{
	obst_avoid::select_theta pub_data;
	pub_data.select_theta=safety_angle;
	pub_data.select_vel=safety_vel;
	pub2.publish(pub_data);
	vfh.draw_best_trajectory(selected_angle_i);
	vfh.publish_velocity(safety_vel,safety_angle);
}
void avoid::publish_grid_map(void)
{
	draw_dangerous_line();
	vfh.set_grid_map_view();
	vfh.publish_grid_map_view();
}
//check intersection and intersection_time
void avoid::draw_dangerous_line(void)
{
	selected_vel=0;
	selected_angle=0;
	set_intersection();
	set_intersection_time();

	float x0,y0,x1,y1,xc,yc;
	float vx,vy;
	float vsize;
	float vxi,vyi;
	float w=0.2;
	for(int i = 0; i < obj_info.objs.size(); i++)
	{
		x0=y0=x1=y1=xc=yc=vx=vy=0;
		if(obstacle_status[i]!=MOVING_OBSTACLE)
		{
			continue;
		}
		xc=x_c[i].x;
		yc=x_c[i].y;
		x0=x_c[i].x-t_c[i]*obj_info.objs[i].vel.x;
		y0=x_c[i].y-t_c[i]*obj_info.objs[i].vel.y;
		x1=x_c[i].x+t_c[i]*obj_info.objs[i].vel.x;
		y1=x_c[i].y+t_c[i]*obj_info.objs[i].vel.y;
		vx=obj_info.objs[i].vel.x;
		vy=obj_info.objs[i].vel.y;
		vsize=std::sqrt(std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.y,2.0));
		vxi=vx/vsize;
		vyi=vy/vsize;
		vfh.draw_line(x0,y0,x1,y1);
		vfh.draw_circle(xc,yc);
		vfh.draw_circle(obj_info.objs[i].pos.x,obj_info.objs[i].pos.z);
		vfh.draw_line(obj_info.objs[i].pos.x,obj_info.objs[i].pos.z,obj_info.objs[i].pos.x+vxi*w,obj_info.objs[i].pos.z+vyi*w);
	}
	//pcl
	show_cross_cloud();
}
void avoid::show_cross_cloud(void)
{
	
	float x0,y0,x1,y1,xc,yc;
	float vx,vy;
	float vsize;
	float vxi,vyi;
	float w=0.2;
	
	//pointcloud
	float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusted_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
	clusted_cloud->points.clear();
	clusted_cloud->points.reserve(673*376*5);
	pcl::PointXYZRGB cloud_temp;
	
	for(int i = 0; i < obj_info.objs.size(); i++)
	{
		x0=y0=x1=y1=xc=yc=vx=vy=0;
		if(obstacle_status[i]!=MOVING_OBSTACLE)
		{
			continue;
		}
		xc=x_c[i].x;
		yc=x_c[i].y;
		x0=x_c[i].x-t_c[i]*obj_info.objs[i].vel.z;
		y0=x_c[i].y-t_c[i]*(-obj_info.objs[i].vel.x);
		x1=x_c[i].x+t_c[i]*obj_info.objs[i].vel.z;
		y1=x_c[i].y+t_c[i]*(-obj_info.objs[i].vel.x);
		vx=obj_info.objs[i].vel.z;
		vy=(-obj_info.objs[i].vel.x);
		vsize=std::sqrt(std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.y,2.0));
		vxi=vx/vsize;
		vyi=vy/vsize;
		//vfh.draw_line(x0,y0,x1,y1);
		//vfh.draw_circle(xc,yc);
		//vfh.draw_circle(obj_info.objs[i].pos.x,obj_info.objs[i].pos.z);
		//vfh.draw_line(obj_info.objs[i].pos.x,obj_info.objs[i].pos.z,obj_info.objs[i].pos.x+vxi*w,obj_info.objs[i].pos.z+vyi*w);
		
//--------point cloud------------------------------------------------

		for(int k = 0; k < obj_info.objs[i].pt.size(); k++)
		{
			double t=0;
			//t=0
			cloud_temp.y=-(obj_info.objs[i].pt[k].x-width/2)*obj_info.objs[i].pt[k].z/f;
			cloud_temp.z=((height/2-obj_info.objs[i].pt[k].y)*obj_info.objs[i].pt[k].z)/f+0.4125;
			cloud_temp.x=obj_info.objs[i].pt[k].z;
			cloud_temp.r=colors[i%12][0];
			cloud_temp.g=colors[i%12][1];
			cloud_temp.b=colors[i%12][2];
			cloud_temp.x+=obj_info.objs[i].vel.z*t;
		    cloud_temp.y+=-obj_info.objs[i].vel.x*t;			
		    cloud_temp.z+=obj_info.objs[i].vel.y*t;
		    
			clusted_cloud->points.push_back(cloud_temp);
			//t=1
			t=1;
			cloud_temp.y=-(obj_info.objs[i].pt[k].x-width/2)*obj_info.objs[i].pt[k].z/f;
			cloud_temp.z=((height/2-obj_info.objs[i].pt[k].y)*obj_info.objs[i].pt[k].z)/f+0.4125;
			cloud_temp.x=obj_info.objs[i].pt[k].z;
			cloud_temp.r=colors[i%12][0];
			cloud_temp.g=colors[i%12][1];
			cloud_temp.b=colors[i%12][2];
			cloud_temp.x+=obj_info.objs[i].vel.z*t;
		    cloud_temp.y+=-obj_info.objs[i].vel.x*t;			
		    cloud_temp.z+=obj_info.objs[i].vel.y*t;
		    
			clusted_cloud->points.push_back(cloud_temp);
		    
		    //t=1
			t=1;
			cloud_temp.y=-(obj_info.objs[i].pt[k].x-width/2)*obj_info.objs[i].pt[k].z/f;
			cloud_temp.z=((height/2-obj_info.objs[i].pt[k].y)*obj_info.objs[i].pt[k].z)/f+0.4125;
			cloud_temp.x=obj_info.objs[i].pt[k].z;
			cloud_temp.r=colors[i%12][0];
			cloud_temp.g=colors[i%12][1];
			cloud_temp.b=colors[i%12][2];
			cloud_temp.x+=obj_info.objs[i].vel.z*t;
		    cloud_temp.y+=-obj_info.objs[i].vel.x*t;			
		    cloud_temp.z+=obj_info.objs[i].vel.y*t;
		    
			clusted_cloud->points.push_back(cloud_temp);
		    
		    //t=t_c_min
			t=t_c_min[i];
			cloud_temp.y=-(obj_info.objs[i].pt[k].x-width/2)*obj_info.objs[i].pt[k].z/f;
			cloud_temp.z=((height/2-obj_info.objs[i].pt[k].y)*obj_info.objs[i].pt[k].z)/f+0.4125;
			cloud_temp.x=obj_info.objs[i].pt[k].z;
			cloud_temp.r=colors[i%12][0];
			cloud_temp.g=colors[i%12][1];
			cloud_temp.b=colors[i%12][2];
			cloud_temp.x+=obj_info.objs[i].vel.z*t;
		    cloud_temp.y+=-obj_info.objs[i].vel.x*t;			
		    cloud_temp.z+=obj_info.objs[i].vel.y*t;
		    
			clusted_cloud->points.push_back(cloud_temp);
		    
		    //t=t_c
			t=t_c[i];
			cloud_temp.y=-(obj_info.objs[i].pt[k].x-width/2)*obj_info.objs[i].pt[k].z/f;
			cloud_temp.z=((height/2-obj_info.objs[i].pt[k].y)*obj_info.objs[i].pt[k].z)/f+0.4125;
			cloud_temp.x=obj_info.objs[i].pt[k].z;
			cloud_temp.r=colors[i%12][0];
			cloud_temp.g=colors[i%12][1];
			cloud_temp.b=colors[i%12][2];
			cloud_temp.x+=obj_info.objs[i].vel.z*t;
		    cloud_temp.y+=-obj_info.objs[i].vel.x*t;			
		    cloud_temp.z+=obj_info.objs[i].vel.y*t;
		    
			clusted_cloud->points.push_back(cloud_temp);
		    
		    //t=t_c
			t=t_c_max[i];
			cloud_temp.y=-(obj_info.objs[i].pt[k].x-width/2)*obj_info.objs[i].pt[k].z/f;
			cloud_temp.z=((height/2-obj_info.objs[i].pt[k].y)*obj_info.objs[i].pt[k].z)/f+0.4125;
			cloud_temp.x=obj_info.objs[i].pt[k].z;
			cloud_temp.r=colors[i%12][0];
			cloud_temp.g=colors[i%12][1];
			cloud_temp.b=colors[i%12][2];
			cloud_temp.x+=obj_info.objs[i].vel.z*t;
		    cloud_temp.y+=-obj_info.objs[i].vel.x*t;			
		    cloud_temp.z+=obj_info.objs[i].vel.y*t;
		    
			clusted_cloud->points.push_back(cloud_temp);
		    
		}		
//---------------------------------------------------------
	}
	
	sensor_msgs::PointCloud2 edit_cloud;
	pcl::toROSMsg (*clusted_cloud, edit_cloud);
	edit_cloud.header.frame_id="/zed_current_frame";
	pub_pcl.publish(edit_cloud);
	
}
int main(int argc,char **argv)
{
  ros::init(argc,argv,"avoid_class_test");

  avoid avoid_cls;
  time_class time_cls;
	int ROBOT_STATUS=ROBOT_GO;
	avoid_cls.set_param_vfh();

	geometry_msgs::Twist turtlebot_vel;
	std_msgs::Empty empty_msg;
	ros::NodeHandle nh,nh2;
	ros::Publisher pub,pub2;
	pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
	pub2 = nh.advertise<std_msgs::Empty>("mobile_base/commands/reset_odometry", 1);

	turtlebot_vel.linear.x=0.2;
	turtlebot_vel.linear.y=0;
	turtlebot_vel.linear.z=0;
	turtlebot_vel.angular.x=0;
	turtlebot_vel.angular.y=0;
	turtlebot_vel.angular.z=0;

	int i=0;
	ros::Rate r=1;

	while(ros::ok()&&i++<2)
	{
		pub2.publish(empty_msg);
		r.sleep();
	}

  while(ros::ok())
	{

		pub.publish(turtlebot_vel);
		ROBOT_STATUS=ROBOT_GO;
		std::cout<<"begin\n";
		avoid_cls.subscribe_objects();
		avoid_cls.subscribe_odometry();
		std::cout<<"subscribed\n";
		//avoid_cls.clear_safety_status();
		if(!avoid_cls.dicriminate_obstacle())
		{
			continue;
		}
		avoid_cls.set_equation();
		std::cout<<"culculate_equation\n";
		avoid_cls.clear_not_select_angle();
		std::cout<<"clear_not_select_angle\n";
		avoid_cls.set_grid_map();
		std::cout<<"set_grid_map\n";
		avoid_cls.set_not_select_angle_to_vfh();
		std::cout<<"set_not_select_angle_to_vfh\n";

		do{

			avoid_cls.select_route();

			std::cout<<"select_route\n";

			if(avoid_cls.select_safety_vel())//if found safety_vel <- exist bag
			{
				std::cout<<"select_safety_vel\n";
				break;
			}
			if(!avoid_cls.set_dangerous_angle())//if nothing safety angle
			{
				ROBOT_STATUS=ROBOT_STOP;
				break;
			}

			/*
			else
			{
				while(ros::ok())
				{
					std::cout<<"Now searching\n";
				}
			}
			*/
		}while(ros::ok());


		if(ROBOT_STATUS==ROBOT_STOP)
		{
			avoid_cls.set_stop_vel();
		}
		else
		{
			avoid_cls.set_safety_vel();
		}

		avoid_cls.publish_velocity();

		avoid_cls.publish_grid_map();


		std::cout<<"loop\n";

	}

	turtlebot_vel.linear.x=0;
	pub.publish(turtlebot_vel);

}
