#include"img_prc_cls.h"

	void ImageProcesser::pub_response(void){
		pub_empty.publish(emptymsg);
	}
	void ImageProcesser::prd_prcess(void){
		//方針
		//分割領域のオプティカルフローの大きさの平均or平均と分散を算出
		//オプティカルフローがない領域なども考慮に入れること
		//領域全体の平均と分散から、平均からより離れている領域に番号付け（ソート）を行い、移動物体領域を検出
		//デバック::より移動物体として認識されている領域に色を加える

		//分割画像の各特徴点の平均と分散(x,y,size)
		cv::Point2d avept[cnh][cnw];	//sum->ave
//		double avesize[cnh][cnw];		//sum->ave
		cv::Point2d dsppt[cnh][cnw];	//sum->dsp
//		double dspsize[cnh][cnw];		//sum->dsp
		double dspz[cnh][cnw];
		double avez[cnh][cnw];
		double p_mvarea[cnh][cnw];
		int pp_mvarea[cnh][cnw];
		double ismvobj[cnh][cnw];
		double ismvline[cnw]={0};
		//initialize

		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				avept[i][j].x=0;
				avept[i][j].y=0;
//				avesize[i][j]=0;
				avez[i][j]=0;
				dsppt[i][j].x=0;
				dsppt[i][j].y=0;
//				dspsize[i][j]=0;
				dspz[i][j]=0;
				ismvobj[i][j]=0;
				pp_mvarea[i][j]=0;
			}
		}
		//calculate sum
		for(int k=0;k<points.size();k++){
			for(int j=0;j<cnw;j++){
				if((int)(j*width/cnw) < (int)points[k].x && (int)points[k].x < (int)((j+1)*width/cnw)){
					for(int i=0;i<cnh;i++){
						if((int)(i*height/cnh)<(int)points[k].y&&(int)points[k].y<(int)((i+1)*height/cnh)){
							if(std::abs(newpoints[k].x-jnewpoints[k].x)*z[k]/f<1.1){
								cpt[i][j].push_back(points[k]);
								cz[i][j].push_back(z[k]);
								cnpt[i][j].push_back(newpoints[k]-jnewpoints[k]+points[k]);
								avept[i][j].x+=newpoints[k].x-jnewpoints[k].x;
								avept[i][j].y+=newpoints[k].y-jnewpoints[k].y;
								avez[i][j]+=z[k];
//							avesize[i][j]+=sqrt(pow(newpoints[k].x-jnewpoints[k].x,2.0)
	//							+pow(newpoints[k].y-jnewpoints[k].y,2.0));
							}
						}
					}
				}
			}
		}
		//calculate average
		double ppT=dt*5;//10;//5;add1114
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				avept[i][j].x=avept[i][j].x/(int)cpt[i][j].size();
				avept[i][j].y=avept[i][j].y/(int)cpt[i][j].size();
				avez[i][j]=avez[i][j]/(int)cpt[i][j].size();
//				if(!std::isnan(pavept[i][j].x)&&!std::isnan(pavept[i][j].x)){
//					avept[i][j].x=(ppT*pavept[i][j].x+dt*avept[i][j].x)/(ppT+dt);
//					avept[i][j].y=(ppT*pavept[i][j].y+dt*avept[i][j].y)/(ppT+dt);
//				}
//				pavept[i][j]=avept[i][j];
//				avesize[i][j]=avesize[i][j]/(int)cpt[i][j].size();
//				avesize[i][j]=sqrt( avept[i][j].x*avept[i][j].x +avept[i][j].y*avept[i][j].y );
//				if(!std::isnan(pavesize[i][j]))
//					avesize[i][j]=(ppT*pavesize[i][j]+dt*avesize[i][j])/(ppT+dt);
//				pavesize[i][j]=avesize[i][j];
			}
		}
		//calculate dispersion
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				for(int k=0;k<cpt[i][j].size();k++){
					dsppt[i][j].x+=(cnpt[i][j][k].x-cpt[i][j][k].x-avept[i][j].x)*(cnpt[i][j][k].x-cpt[i][j][k].x-avept[i][j].x);
//					dsppt[i][j].y+=(cnpt[i][j][k].y-cpt[i][j][k].y-avept[i][j].y)*(cnpt[i][j][k].y-cpt[i][j][k].y-avept[i][j].y);
//					dspsize[i][j]+=pow( sqrt(pow(cnpt[i][j][k].x-cpt[i][j][k].x,2.0)
//							+pow(cnpt[i][j][k].y-cpt[i][j][k].y,2.0)) -avesize[i][j],2.0);
					dspz[i][j]+=pow(cz[i][j][k]-avez[i][j],2.0);
				}
				dsppt[i][j].x=sqrt( dsppt[i][j].x/(int)cpt[i][j].size() );
//				dsppt[i][j].y=sqrt( dsppt[i][j].y/(int)cpt[i][j].size() );
//				dspsize[i][j]=sqrt( dspsize[i][j]/(int)cpt[i][j].size() );
				dspz[i][j]=sqrt( dspz[i][j]/(int)cpt[i][j].size() );
				
			}
		}

		double p_mv;
		double th_dsp;
		double th_mv;
		double pT=dt*5;
//		double ddt=dt/0.045;
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				p_mvarea[i][j]=1;
				th_dsp=1*std::abs(dyaw)/0.01*std::abs(j-cnw/2+0.5);
//				th_mv=2.0*dt/0.10;///ddt;add1114
				th_mv=2.0*dt/0.10;///ddt;add1211
//				th_mv=1.3*dt/0.12;///ddt;add1114
				if(std::abs(dyaw)>0.015)
					th_mv=th_mv*std::abs(dyaw)/0.015;
				if(sp3d.sqr_p3d[i].line_p3d[j].y+0.23<=0||sp3d.sqr_p3d[i].line_p3d[j].y+0.23>1.0
					||std::isnan(avept[i][j].x)
					||std::abs(avept[i][j].x)<th_mv /sp3d.sqr_p3d[i].line_p3d[j].z
//					||sp3d.sqr_p3d[i].line_p3d[j].z>3.0
//					||std::abs(avept[i][j].x)*sp3d.sqr_p3d[i].line_p3d[j].z/f/dt<0.2
					||dsppt[i][j].x>th_mv /sp3d.sqr_p3d[i].line_p3d[j].z			//add
					||dspz[i][j]>0.10					
					||std::abs(avept[i][j].x)*sp3d.sqr_p3d[i].line_p3d[j].z/f/dt>1.00){
					p_mvarea[i][j]=0;
				}
			}
		}

		int mvarea_count=0;
		int all_count=0;
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				if(sp3d.sqr_p3d[i].line_p3d[j].y+0.23>0&&sp3d.sqr_p3d[i].line_p3d[j].y+0.23<=1.0){
					all_count++;
					if(p_mvarea[i][j]!=0)
						mvarea_count++;
				}
			}
		}
		double count_rate=0.3;
		std::cout<<"mvarea_count:"<<mvarea_count<<":\n";	
		if(mvarea_count>all_count*count_rate){
			for(int i=0;i<10;i++)
				std::cout<<"mvarea_count>all_count*"<<count_rate<<":\n";
			return ;
		}
//衝突物体を予測
//avept[i][j].x
		double obj_pstn;
		int prd_obj[cnh][cnw];
		int prd_line[cnw];
		//initialize prd_obj
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				prd_obj[i][j]=0;
			}
		}

		//移動物体領域合成用
		cv::Point2i area_begin;
		cv::Point2i area_end;

		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				if(p_mvarea[i][j]!=0){
					cv::Point2i temp;
					temp.x=j;
					temp.y=i;
					mv_area.push_back(temp);
					area_begin=temp;
					area_end=temp;
					double temp_ave_opt=0;
					int max_prcocess_count=3;
					pp_mvarea[mv_area[0].y][mv_area[0].x]=1;
					for(int k=0;k<mv_area.size();k++){
						if(p_mvarea[mv_area[k].y][mv_area[k].x]!=0){
							opt.push_back(avept[mv_area[k].y][mv_area[k].x].x);
							if(area_begin.x>mv_area[k].x)
								area_begin.x=mv_area[k].x;
							if(area_begin.y>mv_area[k].y)
								area_begin.y=mv_area[k].y;
							if(area_end.x<mv_area[k].x)
								area_end.x=mv_area[k].x;
							if(area_end.y<mv_area[k].y)
								area_end.y=mv_area[k].y;
						}
						int search_range=1;
						for(int l=-search_range;l<=search_range;l++){
							for(int m=-search_range;m<=search_range;m++){
								if(l==0&&m==0)
									continue;
								int u=mv_area[k].x;
								int v=mv_area[k].y;
								if(u+l<0||u+l>cnw-1||v+m<0||v+m>cnh-1)
									continue;
								if(std::isnan(sp3d.sqr_p3d[v+m].line_p3d[u+l].z)||std::isinf(sp3d.sqr_p3d[v+m].line_p3d[u+l].z))
									continue;
								if(sp3d.sqr_p3d[v+m].line_p3d[u+l].y+0.23<=0||sp3d.sqr_p3d[v+m].line_p3d[u+l].y+0.23>1.0)
									continue;
								if(pp_mvarea[v+m][u+l]==1)
									continue;
								if(!std::isnan(avept[v+m][u+l].x)){
									if(avept[v][u].x*avept[v+m][u+l].x>0&&std::abs(avept[v][u].x-avept[v+m][u+l].x)<0.5&&std::abs(avez[v][u]-avez[v+m][u+l])<0.1){
										temp.x=u+l;
										temp.y=v+m;
										pp_mvarea[v+m][u+l]=1;
										mv_area.push_back(temp);
									}
								}
							}
						}
					}
					double ave_opt=0;
					for(int k=0;k<mv_area.size();k++){
						ave_opt+=opt[k];
					}
					ave_opt=ave_opt/(int)opt.size();
//					std::cout<<"mv_area.size():"<<mv_area.size()<<"\n";
//					std::cout<<"area_begin(x,y):end(x,y)("<<area_begin.x<<","<<area_begin.y<<")\n"
//							<<"end("<<area_end.x<<","<<area_end.y<<"\n";
					for(int h=area_begin.y;h<area_end.y+1;h++){
						for(int w=area_begin.x;w<area_end.x+1;w++){
//--
							double obj_pstn;
							if(!std::isnan(avez[h][w]))
//								obj_pstn=ave_opt*avez[h][w]/dz;//
								obj_pstn=ave_opt*avez[h][w]/dr;//
//								obj_pstn=ave_opt*avez[h][w]/(avez[h][w]*(1-cos(dyaw))+dr);;//
							else
//								obj_pstn=ave_opt*sp3d.sqr_p3d[h].line_p3d[w].z/dz;
								obj_pstn=ave_opt*sp3d.sqr_p3d[h].line_p3d[w].z/dr;
//								obj_pstn=ave_opt*sp3d.sqr_p3d[h].line_p3d[w].z/(sp3d.sqr_p3d[h].line_p3d[w].z*(1-cos(dyaw))+dr);
		//					std::cout<<"dz:"<<dz<<"\n";
							obj_pstn=((double)w+0.5)*width/cnw+obj_pstn;
//							std::cout<<"obj_pstn(prev,aftr):("<<((double)w+0.5)*width/cnw<<","<<obj_pstn<<")\n";

							int prd_j=(int)(obj_pstn*cnw/width);
							if(pp_mvarea[h][w]==1)
								std::cout<<"pp_mvarea[h][w]==1\n";
							std::cout<<"obj_pstn(prev,aftr):("<<w<<","<<prd_j<<")\n";
//							std::cout<<"prd_j:"<<prd_j<<"\n";
							//移動するエリアは考慮しない
							if(0<=prd_j&&prd_j<cnw){
								prd_obj[h][prd_j]=1;
								int k_begin;
								int k_end;
								//移動後z
								if(!std::isnan(avez[h][w])){
									if(sp3d.sqr_p3d[h].line_p3d[prd_j].z >avez[h][w]||sp3d.sqr_p3d[h].line_p3d[prd_j].z >sp3d.sqr_p3d[h].line_p3d[w].z){
										if(avez[h][w]<sp3d.sqr_p3d[h].line_p3d[w].z)
											sp3d.sqr_p3d[h].line_p3d[prd_j].z =avez[h][w];
										else
											sp3d.sqr_p3d[h].line_p3d[prd_j].z =sp3d.sqr_p3d[h].line_p3d[w].z;											
//										sp3d.sqr_p3d[h].line_p3d[prd_j].z =avez[h][w]-0.1;
//										if(avez[h][w]<1.5)//add
//											sp3d.sqr_p3d[h].line_p3d[prd_j].z =0.5;

//正規分布
/*
	//移動前後、の間のみ
										if(w<prd_j){
											k_begin=w;
											k_end=prd_j;
										}
										else{
											k_begin=prd_j;
											k_end=w;
										}
										double sgm=(double)(k_end-k_begin)/4;
										for(int k=k_begin;k<=k_end-k_begin;k++){
											double p=exp(-abs(prd_j-k)/(2*sgm*sgm));
											sp3d.sqr_p3d[h].line_p3d[k].z=p*sp3d.sqr_p3d[h].line_p3d[prd_j].z+(1-p)*sp3d.sqr_p3d[h].line_p3d[k].z;
										}
*/
	//全て
/*										double sgm=(double)(std::abs(w-prd_j))/4;
										for(int k=0;k<cnw;k++){
											double p=exp(-abs(prd_j-k)/(2*sgm*sgm));
											sp3d.sqr_p3d[h].line_p3d[k].z=p*sp3d.sqr_p3d[h].line_p3d[prd_j].z+(1-p)*sp3d.sqr_p3d[h].line_p3d[k].z;
										}
*/
									}
								}
								else{
									if(sp3d.sqr_p3d[h].line_p3d[prd_j].z >sp3d.sqr_p3d[h].line_p3d[w].z){
										sp3d.sqr_p3d[h].line_p3d[prd_j].z =sp3d.sqr_p3d[h].line_p3d[w].z;
//										sp3d.sqr_p3d[h].line_p3d[prd_j].z =sp3d.sqr_p3d[h].line_p3d[w].z-0.1;
//										if(sp3d.sqr_p3d[h].line_p3d[w].z<1.5)//add
//										sp3d.sqr_p3d[h].line_p3d[prd_j].z =0.5;

//正規分布
/*
										if(w<prd_j){
											k_begin=w;
											k_end=prd_j;
										}
										else{
											k_begin=prd_j;
											k_end=w;
										}
										double sgm=(double)(k_end-k_begin)/4;
										for(int k=k_begin;k<=k_end-k_begin;k++){
											double p=exp(-abs(prd_j-k)/(2*sgm*sgm));
											sp3d.sqr_p3d[h].line_p3d[k].z=p*sp3d.sqr_p3d[h].line_p3d[prd_j].z+(1-p)*sp3d.sqr_p3d[h].line_p3d[k].z;
										}
*/
	//全て
/*										double sgm=(double)(std::abs(w-prd_j))/4;
										for(int k=0;k<cnw;k++){
											double p=exp(-abs(prd_j-k)/(2*sgm*sgm));
											sp3d.sqr_p3d[h].line_p3d[k].z=p*sp3d.sqr_p3d[h].line_p3d[prd_j].z+(1-p)*sp3d.sqr_p3d[h].line_p3d[k].z;
										}
*/									}
								}
							}
							int color=100;
							for(int u=w*width/cnw;u<(w+1)*width/cnw;u++){
								for(int v=h*height/cnh;v<(h+1)*height/cnh;v++){
									Limg_view.at<cv::Vec3b>(v,u)[1]+=color;
								}
							}
						}
					}
				}
				opt.clear();
				mv_area.clear();
			}
		}
//移動予測エリアを表示
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				//移動後エリアを追加
				if(prd_obj[i][j]!=0){
					int color=100*prd_obj[i][j];
					for(int u=j*width/cnw;u<(j+1)*width/cnw;u++){
						for(int v=i*height/cnh;v<(i+1)*height/cnh;v++){
							Limg_view.at<cv::Vec3b>(v,u)[2]+=color;
						}
					}
				}
			}
		}

		double sum_pmvline[cnw]={0};
		double pmvline[cnw]={0};
		double line_z[cnw]={0};
		int z_count=0;
		for(int j=0;j<cnw;j++){
			double min_z;
			double max_z;
			int nan_count=0;
			int count=0;
			int all_count=0;
			int max_flag=0;
			for(int i=0;i<cnh;i++){
				if(!std::isnan(sp3d.sqr_p3d[i].line_p3d[j].y)){
					if(sp3d.sqr_p3d[i].line_p3d[j].y+0.23>0&&sp3d.sqr_p3d[i].line_p3d[j].y+0.23<1.0){
						if(count==0){
							count++;
							max_z=sp3d.sqr_p3d[i].line_p3d[j].z;
							min_z=sp3d.sqr_p3d[i].line_p3d[j].z;
						}
						if(prd_obj[i][j]==0&&!std::isnan(avez[i][j])){//add1114
/*							int mrgn=(int)(f*0.5/avez[i][j]/width*cnh)+1;///3;add1114
//							std::cout<<"mrgn:"<<mrgn<<"\n";
							bool cnf=false;
							for(int k=-mrgn;k<=mrgn;k++){
								if(j+k<0||j+k>cnh-1){
									continue;
								}
								else{
									if(pp_mvarea[i][j+k]!=0){
										cnf=true;
										break;
									}
								}
							}
							if(cnf)
								continue;
*/
							int mrgn=(int)(f*0.5/avez[i][j]/width*cnh)+1;///3;add1114
							bool cnf=false;
							for(int k=-mrgn;k<=mrgn;k++){
								for(int l=-mrgn;l<=mrgn;l++){
									if(j+k<0||j+k>cnh-1||i+l<0||i+l>cnw-1){
										continue;
									}
									else{
										if(pp_mvarea[i][j+k]!=0){
											cnf=true;
											break;
										}
									}
								}
							}
							if(cnf)
								continue;


						}

						if(pp_mvarea[i][j]==0||prd_obj[i][j]!=0){
							if(min_z>sp3d.sqr_p3d[i].line_p3d[j].z)
								min_z=sp3d.sqr_p3d[i].line_p3d[j].z;
							if(max_z<sp3d.sqr_p3d[i].line_p3d[j].z)
								max_z=sp3d.sqr_p3d[i].line_p3d[j].z;
						}
//						if(p_mvarea[i][j]==1&&prd_obj[i][j]==0)
//							max_flag=1;
					}
				}
				else
					nan_count++;
				all_count++;
			}
//			if(all_count!=0)
//				std::cout<<"line["<<j<<"]:(float)nan_count/all_count:"<<(float)nan_count/all_count<<"\n";
//				std::cout<<"line["<<j<<"]nan_count:"<<nan_count<<"\n";
			if(all_count==0||min_z<0.5)//||min_z>6)//add
				min_z=0.5;
			else if((float)nan_count/all_count>=0.70)
				min_z=0.5;
			else if(min_z!=0.5&&min_z==max_z)
				min_z=0.5;
			if(max_flag==0)
				line_z[j]=min_z;
			else
				line_z[j]=max_z;
			std::cout<<"line["<<j<<"](z,min,max):("<<line_z[j]<<","<<min_z<<","<<max_z<<")"<<"\n";
		}
		//
		double min_line_z=line_z[0];
		for(int j=1;j<cnw;j++){
			if(min_line_z>line_z[j])
				min_line_z=line_z[j];
		}
		const double rw=0.276;//robot width +( left camera position - center position)
		double dif_lens_d=0.075*f/min_line_z/(width/cnw);
		int dif_lens=(int)dif_lens_d+(int)((dif_lens_d-(int)dif_lens_d)*2);//4 out ,5 in
		double w_pix=f*rw/min_line_z;
		int space_minsize=(int)(w_pix/(width/cnw))+1;
//		std::cout<<"dif_lens(i,d):("<<dif_lens<<","<<dif_lens_d<<")\n";
		std::cout<<"space_minsize:"<<space_minsize<<"\n";
		std::vector<int> space_begin;
		std::vector<int> space_end;
		std::vector<int> space_size;
		int space_temp=0;
//set vel and max_vel_dif
		double min_space_z=line_z[cnw/2+(-space_minsize/2)-dif_lens];
		for(int j=-space_minsize/2-dif_lens+1;j<=space_minsize/2+dif_lens;j++){
			if(min_space_z>line_z[cnw/2+(-space_minsize/2)])
					min_space_z=line_z[cnw/2+(-space_minsize/2)];
		}
		if(min_space_z>5)
			min_space_z=5.0;
		max_vel_dif=min_space_z*(-20)+110;
		if(min_space_z<1.0)
			min_space_z=1.0;
//		vel=min_space_z*50+50;
		vel=min_space_z*50+20;

//VFH
		for(int j=0;j<cnw;j++){
			if(line_z[j]>1.0&&!std::isinf(line_z[j])&&!std::isnan(line_z[j]))
				space_temp++;
			else{
				if(space_minsize<=space_temp){
					space_size.push_back(space_temp);
					space_end.push_back(j);
					space_begin.push_back(j-space_temp+1);
				}
				space_temp=0;
			}
			if(j==cnw-1){
				if(space_minsize<=space_temp){
					space_size.push_back(space_temp);
					space_end.push_back(j);
					space_begin.push_back(j-space_temp+1);
				}
			}
		}

		double z_max=5;
		double z_aveline=0;//sum->ave
//		double z_target=0.5;
		int target_num=cnw/2;
		z_target=0.5;

//		for(int i=0;i<cnw;i++)
//			std::cout<<"line_z["<<i<<"]:"<<line_z[i]<<"\n";
//		for(int i=0;i<space_size.size();i++)
//			std::cout<<"space_begin[i],space_size[i]:"<<space_begin[i]<<","<<space_size[i]<<"\n";
		for(int i=0;i<space_size.size();i++){
			//space num
			for(int j=0;j<space_size[i]-(space_minsize-1);j++){
			  double min_space_z=line_z[space_begin[i]+j+0];
			  for(int k=1;k<space_minsize;k++)
			  if(min_space_z>line_z[space_begin[i]+j+k])
			  min_space_z=line_z[space_begin[i]+j+k];
			  z_aveline=min_space_z;
/*				for(int k=0;k<space_minsize;k++)
					z_aveline+=line_z[space_begin[i]+j+k];
				z_aveline=z_aveline/space_minsize;
*/
				if(z_target<z_aveline){
					z_target=z_aveline;
					target_num=space_begin[i]+j+space_minsize/2-dif_lens;
				}
				else if(z_target==z_aveline
					||std::abs(z_target-z_aveline)<z_target*0.1 ){
					if(std::abs(target_num-cnw/2)>std::abs(space_begin[i]+j+space_minsize/2-cnw/2))
						target_num=space_begin[i]+j+space_minsize/2-dif_lens;
				}
				z_aveline=0;
			}
		}


		if((int)space_size.size()==0){
			int right_space=0;
			int left_space=0;
			for(int j=0;j<cnw;j++){
				if(line_z[j]>0.5&&!std::isinf(line_z[j])&&!std::isnan(line_z[j]))
					left_space++;
				else
					break;
			}
			for(int j=cnw-1;j>=0;j--){
				if(line_z[j]>0.5&&!std::isinf(line_z[j])&&!std::isnan(line_z[j]))
					right_space++;
				else
					break;
			}
//		std::cout<<"right_space:"<<right_space<<"\n";
		//スペースがないときは左回転
			if(left_space>=right_space)
				target_num=0;
			else

				target_num=cnw;
		}


		std::cout<<"target_num:"<<target_num<<"\n";
//		std::cout<<"z_target:"<<z_target<<"\n";
		target_point.x=width/cnw*target_num;
//		target_point.x=(pT*ptarget_point.x+dt*target_point.x)/(pT+dt);
		target_point.y=height/2;
		cv::circle(Limg_view, target_point, 4, cv::Scalar(200,200,0),-1, CV_AA);
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				p_pmvarea[i][j]=pp_mvarea[i][j];
				p_avez[i][j]=avez[i][j];
//				p_avez[i][j]=avept[i][j].x*sp3d.sqr_p3d[i].line_p3d[j].z;
						pprd_obj[i][j]=prd_obj[i][j];
			}
		}
		ptarget_point=target_point;
	}

