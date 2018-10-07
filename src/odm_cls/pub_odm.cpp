#include"odometry_class.h"
#include"obst_avoid/point2d.h"
#include"obst_avoid/robot_odm.h"

int main(int argc,char **argv){
	ros::init(argc,argv,"pub_odm");
	odometry_class odom_cls;
	time_class t;
	double t_now=t.get_time_now();

	obst_avoid::robot_odm robot_odm;
	ros::NodeHandle nh;
	ros::Publisher pub;
	pub=nh.advertise<obst_avoid::robot_odm>("robot_odm",1);
	while(ros::ok()){
		odom_cls.subscribe_msgs();
		std::cout<<"subscribed:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		odom_cls.set_data();
		std::cout<<"set_data:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		odom_cls.set_velocity();
		std::cout<<"set_velocity:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		odom_cls.set_odometry();
		std::cout<<"set_odometry:"<<t.get_time_now()-t_now<<"\n";
		t_now=t.get_time_now();
		std::cout<<"v,w:"<<odom_cls.get_velocity()<<","<<odom_cls.get_angular_velocity()<<"\n";
		std::cout<<"r,p,y:"<<odom_cls.get_odometry_r()<<","<<odom_cls.get_odometry_p()<<","<<odom_cls.get_odometry_yw()<<"\n";
		t.set_time();
		std::cout<<"dt:"<<t.get_delta_time()<<"\n";

		robot_odm.x=odom_cls.get_odometry_x();
		robot_odm.y=odom_cls.get_odometry_z();
		robot_odm.th=odom_cls.get_odometry_yw();
		pub.publish(robot_odm);
	}
	
	std::cout<<"finish\n";
	return 0;
}
