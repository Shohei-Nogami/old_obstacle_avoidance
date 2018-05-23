#include"detect_objects_class.h"

void detect_objects::subscribe_opticalflow(void){
	queue_optflw.callOne(ros::WallDuration(1));
}
void detect_objects::opticalflow_callback(const obst_avoid::vel3d::ConstPtr& msg){
	vX.pt=msg->pt;
	vX.vel=msg->vel;
}
void detect_objects::add_velocity_to_cluster(void){
	std::cout<<"vX.pt,vel:"<<vX.pt.size()<<","<<vX.vel.size()<<"\n";
	std::vector< std::vector<pcl::PointXYZ> > cluster_vel;
	std::vector<pcl::PointXYZ> cluster_vel_element;
	pcl::PointXYZ vel_element;
	cluster_vel.resize(cluster.size());
	int h,w;
	int nx,nz,ny;
	int cn;
	for(int k=0;k<vX.pt.size();k++){
		h=vX.pt[k].h;
		w=vX.pt[k].w;
		nx=index_vxl[h][w].nx;
		ny=index_vxl[h][w].ny;
		nz=index_vxl[h][w].nz;
		std::cout<<"nx,ny,nz,ci:"<<nx<<","<<ny<<","<<nz<<","<<clusted_index[nx][nz][nz]<<"\n";
		cn=clusted_index[nx][nz][nz];
		std::cout<<"h,w,,cn:"<<h<<","<<w<<","<<cn<<"\n";
		if(cn!=-1){//cn!=-1
			vel_element.x=vX.vel[k].x;
			vel_element.y=vX.vel[k].y;
			vel_element.z=vX.vel[k].z;

			cluster_vel[cn].push_back(vel_element);
		}

	}
}


