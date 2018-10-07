#include<apf.h>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"apf_test");
	
	//apf
	APF apf(10,10,0.1);
	//or
	//APF apf;
	//apf.set_grid_param(10,10,0.1);//10,10,0.1);
	std::cout<<"wait...\n";
	
	//center point
	cv::Point2f cpt=cv::Point(0.0,0.0);
	//goal point
	cv::Point2f goal_pt=cv::Point2f(7.5-5,7.5-5);
	
	//set_param
	std::cout<<"set_param...\n";
	apf.set_center_point(cpt.x,cpt.y);
	apf.set_goal(goal_pt);
	//--set_robot_param(float x,float y, float r,float vt0,float th_t0)
	if(!apf.set_robot_param(-2.5,-2.5,0.3,0.2,0.0))//-M_PI/2))
	{
		std::cout<<"Error: robot param\n";
		return -1; 
	}
	//--set_command_limit(float dif_vel)
	apf.set_command_limit(0.2);
	
	apf.set_mov_time(0.05);
	//grid_map
	std::cout<<"grid_map...\n";
	cv::Point2f obst_data=cv::Point2f(0.0,0.0);
	cv::Point2f obst_data2=cv::Point2f(2.0,1.8);
	cv::Point2f obst_data3=cv::Point2f(-1.0,1.0);
	cv::Point2f obst_data4=cv::Point2f(1.0,-0.5);
	cv::Point2f obst_data5=cv::Point2f(2.0,-1.0);
	cv::Point2f obst_data6=cv::Point2f(-2.0,1.0);
	cv::Point2f obst_data7=cv::Point2f(3.5,-2.0);
	int obst_num=6;
	//--set_obstacle_data(cv::Point2f& data)
	apf.set_obstacle_data(obst_data);
	apf.set_obstacle_data(obst_data2);
	apf.set_obstacle_data(obst_data3);
	apf.set_obstacle_data(obst_data4);
	apf.set_obstacle_data(obst_data5);
	//apf.set_obstacle_data(obst_data6);
	apf.set_obstacle_data(obst_data7);
	
	//potential_map
	std::cout<<"potential_map...\n";
	apf.create_pot_map();
	
	//gradient_map
	std::cout<<"gradient_map...\n";
	apf.create_grad_map();
	
	//path_planning
	std::cout<<"path_planning...\n";
	apf.draw_path_mat();
		
	std::cout<<"Done\n";

 return 0;
}


