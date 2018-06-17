
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

bool avoid::dicriminate_obstacle(void)
{
	if(!obj_info.objs.size())
	{
		return false;
	}	
	obstacle_status.resize(obj_info.objs.size());
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
	//set grid map
	for (int i = 0; i < obj_info.objs.size(); i++)
	{
		if(obstacle_status[i] != MOVING_OBSTACLE)
		{
			vfh.set_grid_map(obj_info.objs[i].pt);
		}
	}
	vfh.set_grid_map_view();
}
void avoid::select_route(void)
{
	
	//select_route
	cv::Point2f x0 = cv::Point2f(0, 0);
	cv::Point2f xp = cv::Point2f(0, 5);
	float theta0 = 0;
	
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
	//float selected_angle=(min_angle+selected_angle_i*(max_angle-min_angle)/vfh_resolution)*M_PI/180;
	if(selected_angle!=0)
	{
		a_r=1.0/tan(selected_angle);
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
		if(std::abs(a_r-obj_eq_a[i])<M_PI/180*5)//+-5度
		{
			
		  float selected_vel_x=selected_vel*sin(selected_angle);
		  float selected_vel_z=selected_vel*cos(selected_angle);
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
		  	obstacle_status[i]=SAFETY_OBSTACLE;//NO COLLISSION
		  }
		  else
		  {
		    //COLLISSION
		    obstacle_status[i]=DANGER_OBSTACLE;
		    
		  }
		  continue;
		}
		else if(obj_eq_a[i]==0)//not && a_r==0  -> std::abs(a_r-obj_eq_a[i])<M_PI/180*5)
		{
			//float obj_vel=std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0));
			x_c[i].x=obj_info.objs[i].pos.x;
			x_c[i].y=a_r*obj_info.objs[i].pos.x;

		}
		else if(a_r==0)
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
		else if(obstacle_status[i]==DANGER_OBSTACLE)
		{
			float obstacle_vel=std::sqrt(std::pow(obj_info.objs[i].vel.x,2.0)+std::pow(obj_info.objs[i].vel.z,2.0));
		  
		    float length=std::sqrt(std::pow(obj_info.objs[i].pos.x,2.0)+std::pow(obj_info.objs[i].pos.z,2.0));
		    t_c[i] = length/(selected_vel-obstacle_vel);
		    
		}
	}
	
}
void avoid::labeling_dangerous_obstacle(void)
{
	
	for(int i = 0; i < obj_info.objs.size(); i++)
	{
		if(obstacle_status[i]==MOVING_OBSTACLE||obstacle_status[i]==DANGER_OBSTACLE)
	  {
			std::cout<<"t_c["<<i<<"]:"<<t_c[i]<<"\n";
		}
	  if(obstacle_status[i]==MOVING_OBSTACLE)
	  {
	    if(t_c[i]<0||t_c[i]>10)
	    {
	      obstacle_status[i]=SAFETY_OBSTACLE;
	    }
	    else
	    {
	      obstacle_status[i]=WARNING_OBSTACLE;
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
  	if(obstacle_status[i]!=WARNING_OBSTACLE)
  	{
  		continue;
  	}
  	
    double obstacle_r=obj_info.objs[i].r;
	
		double l=(obstacle_r+robot_r)/(tan(selected_angle));
	
		double delta_t=l/selected_vel;
	
		t_c_min[i]=t_c[i]-delta_t;
		t_c_max[i]=t_c[i]+delta_t;
		std::cout<<"t_c_min["<<i<<"]:"<<t_c_min[i]<<"\n";
		std::cout<<"t_c_max["<<i<<"]:"<<t_c_max[i]<<"\n";
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
	  if(obstacle_status[i]!=WARNING_OBSTACLE)
	  {
			if(obstacle_status[i]==DANGER_OBSTACLE)
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
	    obstacle_status[i]=DANGER_OBSTACLE;
	  }
	  else
	  {
	    obstacle_status[i]=SAFETY_OBSTACLE;
	  }
	}
	//std::cout<<"t_c_min_angle.size:"<<t_c_min_angle.size()<<"\n";
	//std::cout<<"selected_angle_i:"<<selected_angle_i<<"\n";
	//std::cout<<"t_c_min_angle[selected_angle_i].size():"<<t_c_min_angle[selected_angle_i].size()<<"\n";
	//std::cout<<"selected_vel_num:"<<selected_vel_num<<"\n";
	if(t_c_min_all!=100)
	{
		t_c_min_angle[selected_angle_i][selected_vel_num]=t_c_min_all;
		return true;
	}
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
}
void avoid::set_stop_vel(void)
{
	safety_vel=0;
	safety_angle=0;	
}

void avoid::publish_velocity(void)
{
	vfh.draw_best_trajectory(selected_angle_i);
	vfh.publish_velocity(safety_vel,safety_angle);
}
void avoid::publish_grid_map(void)
{
	vfh.publish_grid_map_view();
}
int main(int argc,char **argv)
{
  ros::init(argc,argv,"avoid_class_test");
 
  avoid avoid_cls;
  time_class time_cls;
	
	int ROBOT_STATUS=ROBOT_GO;
	avoid_cls.set_param_vfh();
  while(ros::ok())
	{
		ROBOT_STATUS=ROBOT_GO;
		std::cout<<"begin\n";
		avoid_cls.subscribe_objects();
		std::cout<<"subscribed\n";
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
}

