#include"detect_objects_class.h"

cv::Mat& detect_objects::conv_depth_image(cv::Mat& tmp_img_dpt){
	int H=tmp_img_dpt.raw();
	int W=tmp_img_dpt.col();
	int chd=tmp_img_dpt.channels();
	int ch3d=img_3d.channels();
	//cv::Mat img_3d

	for(int h=0;h<H;h++){
		float *pd = tmp_img_dpt.ptr<float>(h);
		float *p3d = img_3d.ptr<cv::Vec3f>(h);
		for(int w=0;w<W;w++){
			if(std::isnan(pd[w*chf])){
				p3d[w*ch3d][0]=-1;//X
				p3d[w*ch3d][1]=-1;//Y
				p3d[w*ch3d][2]=-1;//Z
			}
			else{
				p3d[w*ch3d][0]=pd[w*chf];//X
				p3d[w*ch3d][1]=(w-W/2)*pd[w*chf]/f;//Y
				p3d[w*ch3d][2]=(H-h)*pd[w*chf]/f;//Z
			}
		}
	}
	return img_3d;
}
bool detect_objects::convert_xyz_to_grid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標
	float cx=0;
	float cy=0;
	//APF classから引用
	float map_wf=10;
	float map_hf=10;
	float reso=0.1;

	float map_ptx = map_wf/2 + (pt.x - cx);
	float map_pty = map_hf/2 + ( -(pt.y - cy) );
	if(map_ptx<0 || map_ptx>map_wf)
		return false;

	if(map_pty<0 || map_pty>map_hf)
		return false;

	xg =	(int)(map_ptx/reso);
	yg =	(int)(map_pty/reso);

	return true;
}
cv::Mat& detect_objects::create_grid_map(cv::Mat& tmp_img_3d){

	int H=tmp_img_3d.col();
	int W=tmp_img_3d.raw();
	int ch3d=tmp_img_3d.channels();

	cv::Mat index_to_gm=cv::Mat::zeros(cv::Size(H,W), CV_32SC2);//in header
	int ch2i=index_to_gm.channels();
	//cv::Mat grid_map=cv::Mat::zeros(cv::Size(H,W), CV_32SC1);;
	int chg=grid_map.channels();
	for(int h=0;h<H;h++){
		float *p3d = tmp_img_3d.ptr<cv::Vec3f>(h);
		int *p2i = index_to_gm.ptr<cv::Vec2i>(h);
		for(int w=0;w<W;w++){
			float xg,yg;
			if(p3d[w*ch3d][0]<0){//is nan
				continue;
			}
			if(convert_xy_to_grid(p3d[w*ch3d][0],p3d[w*ch3d][1],xg,yg)){
				//increment grid cell
				int *pg=grid_map.ptr<int>(yg);
				pg[xg*chg]++;
				//set index
				p2i[w*ch2i][0]=xg;
				p2i[w*ch2i][1]=yg;
			}
		}
	}
	return grid_map;
}
void dbscan_with_gm(cv::Mat& tmp_grid_map){

	//grid map
	int H=tmp_grid_map.col();
	int W=tmp_grid_map.raw();
	int chg=tmp_grid_map.channels();
	//searched flag (set cluster num )
	cv::Mat cluster_num=cv::Mat::zeros(cv::Size(H,W), CV_32SC1);
	cluster_num-=1;
	int cluster_size=0;
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
							/*
							int *pg_ks=grid_map.ptr<int>(h_ks);
							*/
							//2
							int *pg_ks;
							int *pc_ks;
							if(!w_ks){
								*pg_ks=grid_map.ptr<int>(hs);
								*pc_ks=cluster_num.ptr<int>(hs);
							}
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
						int hs=;
						int ws=w_ks-sr/2+w;
						int *pc_ks=cluster_num.ptr<int>(task_point[1][k]);
						pc[task_point[0][k]*chc]=cluster_size;
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
}

void detect_objects::draw_grid_map(cv::Mat& tmp_grid_map){
	
	int H=tmp_grid_map.col();
	int W=tmp_grid_map.raw();
	int chg=tmp_grid_map.channels();
	cv::Mat grid_color_img==cv::Mat::zeros(cv::Size(H,W), CV_8UC3);
	int chgc=grid_color_img.channels();
	
	for(int h=0;h<H;h++){
		int *pg=grid_map.ptr<int>(h);
		int *pgc=grid_color_img.ptr<cv::Vec3b>(h);
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

