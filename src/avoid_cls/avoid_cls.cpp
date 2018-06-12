#include"avoid_cls.h"

#define STATIC_OBSTACLE 0
#define DYNAMIC_OBSTACLE 1
#define MOVING_OBSTACLE 2

avoid::avoid()
{
	nh_sub.setCallbackQueue(&queue);
	sub = nh_sub.subscribe("/filted_objects_info", 1, &avoid::objects_callback, this);

	//vfh.vfh_class();

}
avoid::~avoid()
{
	//vfh.~vfh_class();
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


		if (obj_info.objs[i].size > 1.0*1.0 || obj_info.objs[i].size < 0.2*0.2
			|| std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) < 0.1
			)
		{
			obstacle_status[i] = STATIC_OBSTACLE;
		}
		else if (std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) > 1.1
			|| std::sqrt(obj_info.objs[i].dsp.x + obj_info.objs[i].dsp.z) 
				> std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) )
		{
			obstacle_status[i] = DYNAMIC_OBSTACLE;
		}
		else
		{
			obstacle_status[i] = MOVING_OBSTACLE;
		}
		
	}
}

void avoid::culculate_equation(void)
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

float avoid::select_route(void)
{
	//set grid map
	for (int i = 0; i < obj_info.objs.size(); i++)
	{
		vfh.set_grid_map(obj_info.objs[i].pt);
	}
	//select_route
	cv::Point2f x0 = cv::Point2f(0, 0);
	cv::Point2f xp = cv::Point2f(0, 5);
	float theta0 = 0;
	float w_target = 0.8;
	float w_angle = 0.5;
	float selected_angle = vfh.select_best_trajectory(x0, theta0, xp, w_target, w_angle);
	
	selected_angle_i=selected_angle;

	return selected_angle;
	
}

void avoid::culculate_intersection(void)
{
	float a_r;
	if(selected_angle_i!=0)
	{
		a_r=1.0/tan(selected_angle_i);
	}
	else
	{
		a_r=0;
	}
	std::vector<cv::Point2f> x_c;
	std::vector<double> t_c;
	x_c.resize(obj_info.objs.size());
	t_c.resize(obj_info.objs.size());
	for(int i = 0; i < obj_info.objs.size(); i++)
	{
		if(obstacle_status[i]!=MOVING_OBSTACLE||std::abs(a_r-obj_eq_a[i])<M_PI/180*5)
		{
			continue;
		}
		if(obj_eq_a[i]==0)//not && a_r==0  -> std::abs(a_r-obj_eq_a[i])<M_PI/180*5)
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
	}
	
}
int main(int argc,char **argv)
{
  ros::init(argc,argv,"avoid_class_test");
 
  avoid avoid_cls;
  time_class time_cls;
  while(ros::ok())
	{
		std::cout<<"begin\n";
		avoid_cls.subscribe_objects();
		std::cout<<"subscribed\n";
		if(!avoid_cls.dicriminate_obstacle())
		{
			continue;
		}
		avoid_cls.culculate_equation();
		std::cout<<"culculate_equation\n";
		
		
		std::cout<<"loop\n";
		
	}
}
