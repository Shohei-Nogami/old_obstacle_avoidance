#include"detect_objects_class.h"

void detect_objects::conv_depth_image(cv::Mat& tmp_img_dpt){
	int H=tmp_img_dpt.rows;
	int W=tmp_img_dpt.cols;
	int chd=tmp_img_dpt.channels();
	int ch3d=img_3d.channels();
	//cv::Mat img_3d

	for(int h=0;h<H;h++){
		float *pd = tmp_img_dpt.ptr<float>(h);
		cv::Vec3f *p3d = img_3d.ptr<cv::Vec3f>(h);
		for(int w=0;w<W;w++){
			if(std::isnan(pd[w*chd])){
				p3d[w*ch3d][0]=-1;//X
				p3d[w*ch3d][1]=-1;//Y
				p3d[w*ch3d][2]=-1;//Z
			}
			else{
				p3d[w*ch3d][0]=pd[w*chd];//X
				p3d[w*ch3d][1]=(w-W/2)*pd[w*chd]/f;//Y
				p3d[w*ch3d][2]=(H-h)*pd[w*chd]/f;//Z
			}
		}
	}
}
void detect_objects::conv_depth_image(void){
	int H=depth_image.rows;
	int W=depth_image.cols;
	int chd=depth_image.channels();
	int ch3d=img_3d.channels();
	//cv::Mat img_3d
	for(int h=0;h<H;h++){
		float *pd = depth_image.ptr<float>(h);
		cv::Vec3f *p3d = img_3d.ptr<cv::Vec3f>(h);
		for(int w=0;w<W;w++){
			if(std::isnan(pd[w*chd])||std::isinf(pd[w*chd])){
				p3d[w*ch3d][0]=-1;//X
				p3d[w*ch3d][1]=-1;//Y
				p3d[w*ch3d][2]=-1;//Z
			}
			else{
				p3d[w*ch3d][0]=pd[w*chd];//X
				p3d[w*ch3d][1]=(w-W/2)*pd[w*chd]/f;//Y
				p3d[w*ch3d][2]=(H-h)*pd[w*chd]/f;//Z
			}
		}
	}
}

bool detect_objects::convert_xyz_to_grid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標
	/*
	float cx=0;
	float cy=0;
	//APF classから引用
	float map_wf=10;
	float map_hf=10;
	float reso=0.1;
	*/
	float map_ptx = map_wf/2 + (x - cx);
	float map_pty = map_hf/2 + ( -(y - cy) );
	if(map_ptx<0 || map_ptx>map_wf)
		return false;

	if(map_pty<0 || map_pty>map_hf)
		return false;

	xg =	(int)(map_ptx/reso);
	yg =	(int)(map_pty/reso);

	return true;
}
void detect_objects::create_grid_map(cv::Mat& tmp_img_3d){

	int H=tmp_img_3d.rows;
	int W=tmp_img_3d.cols;
	int ch3d=tmp_img_3d.channels();

	index_to_gm=cv::Mat::zeros(cv::Size(W,H), CV_32SC2);//in header

	int ch2i=index_to_gm.channels();
	//cv::Mat grid_map=cv::Mat::zeros(cv::Size(W,H), CV_32SC1);;
	grid_map=cv::Mat::zeros(cv::Size((int)(map_wf/reso),(int)(map_hf/reso)), CV_32SC1);
	int chg=grid_map.channels();
	for(int h=0;h<H;h++){
		cv::Vec3f *p3d = tmp_img_3d.ptr<cv::Vec3f>(h);
		cv::Vec2i *p2i = index_to_gm.ptr<cv::Vec2i>(h);
		for(int w=0;w<W;w++){
			int xg,yg;
			if(p3d[w*ch3d][0]<0){//is nan
				continue;
			}
			if(convert_xyz_to_grid(p3d[w*ch3d][0],p3d[w*ch3d][1],xg,yg)){
				//increment grid cell
				int *pg=grid_map.ptr<int>(yg);
				pg[xg*chg]++;
				//set index
				p2i[w*ch2i][0]=xg;
				p2i[w*ch2i][1]=yg;
			}
			else{
				p2i[w*ch2i][0]=0;
				p2i[w*ch2i][1]=0;
			}
		}
	}
}
void detect_objects::create_grid_map(void){

	int H=img_3d.rows;
	int W=img_3d.cols;
	int ch3d=img_3d.channels();

	index_to_gm=cv::Mat::zeros(cv::Size(W,H), CV_32SC2);//in header

	int ch2i=index_to_gm.channels();
	//cv::Mat grid_map=cv::Mat::zeros(cv::Size(W,H), CV_32SC1);;
	int chg=grid_map.channels();
	for(int h=0;h<H;h++){
		cv::Vec3f *p3d = img_3d.ptr<cv::Vec3f>(h);
		cv::Vec2i *p2i = index_to_gm.ptr<cv::Vec2i>(h);
		for(int w=0;w<W;w++){
			int xg,yg;
			if(p3d[w*ch3d][0]<0){//is nan
				continue;
			}
			if(convert_xyz_to_grid(p3d[w*ch3d][0],p3d[w*ch3d][1],xg,yg)){
				//increment grid cell
				int *pg=grid_map.ptr<int>(yg);
				pg[xg*chg]++;
				//set index
				p2i[w*ch2i][0]=xg;
				p2i[w*ch2i][1]=yg;
			}
			else{
				p2i[w*ch2i][0]=0;
				p2i[w*ch2i][1]=0;
			}
		}
	}
}

void detect_objects::dbscan_with_gm(cv::Mat& tmp_grid_map){

	//grid map
	int H=tmp_grid_map.rows;
	int W=tmp_grid_map.cols;
	int chg=tmp_grid_map.channels();
	//searched flag (set cluster num )
	// cv::Mat cluster_num=cv::Mat::zeros(cv::Size(W,H), CV_32SC1);
	cluster_num=cv::Mat::zeros(cv::Size(W,H), CV_32SC1);
	cluster_num-=1;
	// int cluster_size=0;
	cluster_size=0;
	cluster_count.clear();
	cluster_count.resize(H*W);
	int chc=cluster_num.channels();
	//task points
	std::vector<int> task_point[2];//0:x,1:y
	task_point[0].resize(H*W);
	task_point[1].resize(H*W);
	int task_size=0;
	//
	for(int h=0;h<H;h++){
		int *pg=grid_map.ptr<int>(h);
		int *pc=cluster_num.ptr<int>(h);
		for(int w=0;w<W;w++){
			//if searched
			if(pc[w*chc]>=0){
				continue;
			}
			//
			if(pg[w*chg]>0)
			{
				//culc density
				float reso=0.1;
				float search_range=0.2;
				int sr=(int)(search_range/reso);
				//set task point
				task_size=0;
				task_point[0][task_size]=w;
				task_point[1][task_size++]=h;
				//set cluster_num
				pc[w*chc]=cluster_size;
				for(int k=0;k<task_size;k++){
					//candidate points
					int cand_num[sr*sr];
					int cand_size=0;
					//density
					int dens=0;
					//search r_max

					//2
					// int *pg_ks;
					// int *pc_ks;
					for(int ks=0;ks<sr*sr;ks++){
						int h_ks=ks/sr;
						int w_ks=ks%sr;
						int hs=h_ks-sr/2+h;
						int ws=w_ks-sr/2+w;
						//select 2 ways
						//1
						int *pg_ks=grid_map.ptr<int>(hs);
						int	*pc_ks=cluster_num.ptr<int>(hs);
						//2
						// if(!w_ks){
						// 	*pg_ks=grid_map.ptr<int>(hs);
						// 	*pc_ks=cluster_num.ptr<int>(hs);
						// }
						//
						if(pg_ks[ws*chg]>0){
							//add density
							dens+=pg_ks[ws*chg];
							//record candidate point
							if(!pc_ks[ws*chc]){
								cand_num[cand_size++]=ks;
							}
						}
					}
					//check density value
					//density_threshold
					//float win_size=reso*reso*ks;
					//float density=dens/win_size;
					//float density_th=10.0/(0.1*0.1);
					int density_th_i=500;//temp
					if(dens>density_th_i){
						//true
						//add candidate points to task points
						for(int ks=0;ks<cand_size;ks++){
							task_point[0][task_size]=ks%sr-sr/2+w;
							task_point[1][task_size++]=ks/sr-sr/2+h;
						}
						//add searching point to cluster
						//-->set cluster number to
						//int hs=h_ks-sr/2+h;
						//int ws=w_ks-sr/2+w;
						int *pc_ks=cluster_num.ptr<int>(task_point[1][k]);
						pc_ks[task_point[0][k]*chc]=cluster_size;
						cluster_count[cluster_size]++;
					}
					else{
						//false
						continue;
					}
				}//task loop end

				//All task were searched
				cluster_size++;
			}
		}
	}
	cluster_count.resize(cluster_size);
}

void detect_objects::dbscan_with_gm(void){

	//grid map
	int H=grid_map.rows;
	int W=grid_map.cols;
	int chg=grid_map.channels();
	//searched flag (set cluster num )
	// cv::Mat cluster_num=cv::Mat::zeros(cv::Size(W,H), CV_32SC1);
	cluster_num=cv::Mat::zeros(cv::Size(W,H), CV_32SC1);
	cluster_num-=1;
	// int cluster_size=0;
	cluster_size=0;
	cluster_count.clear();
	cluster_count.resize(H*W);
	for(int k=0;k<H*W;k++){
		cluster_count[k]=0;
	}
	int chc=cluster_num.channels();
	//task points
	std::vector<int> task_point[2];//0:x,1:y
	task_point[0].resize(H*W);
	task_point[1].resize(H*W);
	int task_size=0;
	//
	for(int h=0;h<H;h++){
		int *pg=grid_map.ptr<int>(h);
		int *pc=cluster_num.ptr<int>(h);
		for(int w=0;w<W;w++){
			//if searched
			if(pc[w*chc]>=0){
				continue;
			}
			//
			if(pg[w*chg]>0)
			{
				//culc density
				float reso=0.1;
				float search_range=0.2;
				int sr=(int)(search_range/reso);
				//set task point
				task_size=0;
				task_point[0][task_size]=w;
				task_point[1][task_size++]=h;
				//set cluster_num
				pc[w*chc]=cluster_size;
				for(int k=0;k<task_size;k++){
					//candidate points
					int cand_num[sr*sr];
					int cand_size=0;
					//density
					int dens=0;
					//search r_max
					for(int ks=0;ks<sr*sr;ks++){
							int h_ks=ks/sr;
							int w_ks=ks%sr;
							int hs=h_ks-sr/2+h;
							int ws=w_ks-sr/2+w;
							//select 2 ways
							//1
							int *pg_ks=grid_map.ptr<int>(hs);
							int *pc_ks=cluster_num.ptr<int>(hs);
							//2
							/*int *pg_ks;
							int *pc_ks;
							if(!w_ks){
								*pg_ks=grid_map.ptr<int>(hs);
								*pc_ks=cluster_num.ptr<int>(hs);
							}
							*/
							//
							if(pg_ks[ws*chg]>0){
								//add density
								dens+=pg_ks[ws*chg];
								//record candidate point
								if(!pc_ks[ws*chc]){
									cand_num[cand_size++]=ks;
								}
							}
					}
					//check density value
					//density_threshold
					//float win_size=reso*reso*ks;
					//float density=dens/win_size;
					//float density_th=10.0/(0.1*0.1);
					int density_th_i=500;//temp
					if(dens>density_th_i){
						//true
						//add candidate points to task points
						for(int ks=0;ks<cand_size;ks++){
							task_point[0][task_size]=ks%sr-sr/2+w;
							task_point[1][task_size++]=ks/sr-sr/2+h;
						}
						//add searching point to cluster
						//-->set cluster number to
						// int hs=;
						// int ws=w_ks-sr/2+w;
						int *pc_ks=cluster_num.ptr<int>(task_point[1][k]);
						pc_ks[task_point[0][k]*chc]=cluster_size;
						cluster_count[cluster_size]+=dens;
					}
					else{
						//false
						continue;
					}
				}//task loop end

				//All task were searched
				cluster_size++;
			}
		}
	}
	cluster_count.resize(cluster_size);
}

cv::Mat& detect_objects::get_grid_map(void){
	return grid_map;
}

void detect_objects::set_cluster(void){
	std::vector<int> cluster_k;
	cluster_k.resize(cluster_size);
	for(int k=0;k<cluster_size;k++){
		cluster_k[k]=0;
	}
	//cluster
	for(int i=0;i<Q.clst.size();i++){
		Q.clst[i].pt.clear();
		Q.clst[i].fpt.clear();
	}
	Q.clst.clear();
	//reserve memory
	// Q.clst.reserve(width*height);
	// Q_p.pt.reserve(width*height);
	// Q_p.fpt.reserve(width*height);
	Q.clst.resize(cluster_size);
	for(int i=0;i<Q.clst.size();i++){
		Q.clst[i].pt.resize(cluster_count[i]);
		Q.clst[i].fpt.resize(cluster_count[i]);
	}

	int H=img_3d.cols;
	int W=img_3d.rows;
	std::cout<<"in set_cluster\n";
	int ch3d=img_3d.channels();
	int ch2i=index_to_gm.channels();
	int chcn=cluster_num.channels();
	for(int h=0;h<H;h++){
		cv::Vec3f *p3d = img_3d.ptr<cv::Vec3f>(h);
		cv::Vec2i *p2i = index_to_gm.ptr<cv::Vec2i>(h);
		for(int w=0;w<W;w++){
			//Be able to convert to grid map cell
			if(p2i[w*ch2i][0]!=0 || p2i[w*ch2i][1]!=0){
				//get cluster num
				int *pc=cluster_num.ptr<int>(p2i[w*ch2i][1]);
				int cn=pc[p2i[w*ch2i][0]*chcn];
				std::cout<<"("<<w<<","<<h<<","<<p3d[w*ch3d][0]<<"):"
					<<"("<<cn<<","<<cluster_k[cn]<<"):("
					<<cluster_k.size()<<","<<cluster_count[cn]<<")\n";
				Q.clst[cn].pt[cluster_k[cn]].x=w;
				Q.clst[cn].pt[cluster_k[cn]].y=h;
				Q.clst[cn].pt[cluster_k[cn]].z=p3d[w*ch3d][0];
				cluster_k[cn]++;
			}
		}
	}
}
void detect_objects::draw_grid_map(cv::Mat& tmp_grid_map){

	int H=tmp_grid_map.rows;
	int W=tmp_grid_map.cols;
	int chg=tmp_grid_map.channels();
	cv::Mat grid_color_img=cv::Mat::zeros(cv::Size(W,H), CV_8UC3);
	int chgc=grid_color_img.channels();

	for(int h=0;h<H;h++){
		int *pg=grid_map.ptr<int>(h);
		cv::Vec3b *pgc=grid_color_img.ptr<cv::Vec3b>(h);
		for(int w=0;w<W;w++){
			//init
			pgc[w*chgc][0]=0;
			pgc[w*chgc][1]=0;
			pgc[w*chgc][2]=0;
			//RGB>GB>B>black:765>510>255>0
			for(int color=0;color<pg[w*chg]/3;color++){
				//color:0,1,2-->B,G,R
				pgc[w*chgc][color]=pg[w*chg]-255*color;
			}

		}
	}
}
