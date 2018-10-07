#include"visual_odometry_class.h"
#include"wheel_odometry_class.h"

class odometry_class{
	private:
		double x,y,z;
		double r,p,yw;
		
		double wv,ww;
		double wv_x,wv_y,wv_z;
		double wv_size;
		
		double vv,vw;
		double vv_x,vv_y,vv_z;
		double vv_size;
		
		double ip;
		double dif_angle;
		const double th_angle;
		
		double v,w;
		time_class tm_odm;
		wheel_odometry_class wocls;
		visual_odometry_class vocls;
	public:
		odometry_class();
		~odometry_class();
		
		void subscribe_msgs(void);
		
		void set_data(void);
		void set_velocity(void);
		void set_odometry(void);
		
		double& get_dif_angle(void);
		
		double& get_angular_velocity(void);
		double& get_velocity(void);
		double& get_odometry_x(void);
		double& get_odometry_y(void);
		double& get_odometry_z(void);
		double& get_odometry_r(void);
		double& get_odometry_p(void);
		double& get_odometry_yw(void);
};


