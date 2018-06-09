#include"avoid_cls.h"

#define STATIC_OBSTACLE 0
#define DYNAMIC_OBSTACLE 1
#define MOVING_OBSTACLE 2

avoid::avoid()
{
	nh_sub.setCallbackQueue(&queue);
	sub = nh_sub.subscribe("/filted_objects_info", 1, &avoid::objects_callback, this);
}
avoid::~avoid()
{

}
void avoid::subscribe_objects(void)
{
	queue.callOne(ros::WallDuration(1));
}

void avoid::objects_callback(const obst_avoid::filted_objects_info::ConstPtr& msg)
{
	obj_info.objs = msg->objs;
}

void avoid::dicriminate_obstacle(void)
{
	is_moving_obstacle.resize(obj_info.obj.size());
	for (int i = 0; i < obj_info.obj.size(); i++)
	{


		if (obj_info.objs[i].size > 1.0*1.0 || obj_info.objs[i].size < 0.2*0.2
			|| std::sqrt(std::pow(xh_t[i](2, 0), 2.0) + std::pow(xh_t[i](3, 0), 2.0)) < 0.1
			)
		{
			is_moving_obstacle[i] = STATIC_OBSTACLE;
		}
		else if (std::sqrt(std::pow(obj_info.objs[i].vel.x, 2.0) + std::pow(obj_info.objs[i].vel.z, 2.0)) > 1.1
			|| std::sqrt(std::pow(sig_xh_t[i](2, 2), 2.0) + std::pow(sig_xh_t[i](2, 0), 2.0)) < xh_t[i](2, 0) )
		{
			is_moving_obstacle[i] = DYNAMIC_OBSTACLE;
		}
		else
		{
			is_moving_obstacle[i] = MOVING_OBSTACLE;
		}
		
	}
}

void avoid::culculate_equation(void)
{
	obj_eq_a.clear();
	obj_eq_a.resize(obj_info.obj.size());

	for (int i = 0; i < obj_eq.size(); i++)
	{
		obj_eq_a[i] = obj_info.obj[i].vel.z / obj_info.obj[i].vel.x;
	}
}

float avoid::select_route(void)
{
	//set grid map
	for (int i = 0; i < obj_info.obj.size(); i++)
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

	return selected_angle;

}



