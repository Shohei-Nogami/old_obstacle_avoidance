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
		double avesize[cnh][cnw];		//sum->ave
		cv::Point2d dsppt[cnh][cnw];	//sum->dsp
		double dspsize[cnh][cnw];		//sum->dsp
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
				avesize[i][j]=0;
				avez[i][j]=0;
				dsppt[i][j].x=0;
				dsppt[i][j].y=0;
				dspsize[i][j]=0;
				ismvobj[i][j]=0;
			}
		}
		//calculate sum
		for(int k=0;k<points.size();k++){
			for(int j=0;j<cnw;j++){
				if((int)(j*width/cnw) < (int)points[k].x && (int)points[k].x < (int)((j+1)*width/cnw)){
					for(int i=0;i<cnh;i++){
						if((int)(i*height/cnh)<(int)points[k].y&&(int)points[k].y<(int)((i+1)*height/cnh)){
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
		//calculate average
		double ppT=dt*10;//5;
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				avept[i][j].x=avept[i][j].x/(int)cpt[i][j].size();
				avept[i][j].y=avept[i][j].y/(int)cpt[i][j].size();
				avez[i][j]=avez[i][j]/(int)cpt[i][j].size();
				if(!std::isnan(pavept[i][j].x)&&!std::isnan(pavept[i][j].x)){
					avept[i][j].x=(ppT*pavept[i][j].x+dt*avept[i][j].x)/(ppT+dt);
					avept[i][j].y=(ppT*pavept[i][j].y+dt*avept[i][j].y)/(ppT+dt);
				}
				pavept[i][j]=avept[i][j];
//				avesize[i][j]=avesize[i][j]/(int)cpt[i][j].size();
				avesize[i][j]=sqrt( avept[i][j].x*avept[i][j].x +avept[i][j].y*avept[i][j].y );
				if(!std::isnan(pavesize[i][j]))
					avesize[i][j]=(ppT*pavesize[i][j]+dt*avesize[i][j])/(ppT+dt);
				pavesize[i][j]=avesize[i][j];
			}
		}
		//calculate dispersion
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				for(int k=0;k<cpt[i][j].size();k++){
					dsppt[i][j].x+=(cnpt[i][j][k].x-cpt[i][j][k].x-avept[i][j].x)*(cnpt[i][j][k].x-cpt[i][j][k].x-avept[i][j].x);
					dsppt[i][j].y+=(cnpt[i][j][k].y-cpt[i][j][k].y-avept[i][j].y)*(cnpt[i][j][k].y-cpt[i][j][k].y-avept[i][j].y);
					dspsize[i][j]+=pow( sqrt(pow(cnpt[i][j][k].x-cpt[i][j][k].x,2.0)
							+pow(cnpt[i][j][k].y-cpt[i][j][k].y,2.0)) -avesize[i][j],2.0);
				}
				dsppt[i][j].x=sqrt( dsppt[i][j].x/(int)cpt[i][j].size() );
				dsppt[i][j].y=sqrt( dsppt[i][j].y/(int)cpt[i][j].size() );
				dspsize[i][j]=sqrt( dspsize[i][j]/(int)cpt[i][j].size() );
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
				th_mv=1.2;///ddt;
				if(std::abs(dyaw)>0.010)
					th_mv=th_mv*std::abs(dyaw)/0.010;
				if(sp3d.sqr_p3d[i].line_p3d[j].y+0.23<=0||sp3d.sqr_p3d[i].line_p3d[j].y+0.23>1.0
					||std::isnan(avept[i][j].x)
					||std::abs(avept[i][j].x)<th_mv/sp3d.sqr_p3d[i].line_p3d[j].z
					||th_mv<dsppt[i][j].x){
					p_mvarea[i][j]=0;
				}
//
//				p_mvarea[i][j]=(dt*p_mvarea[i][j]+pT*p_pmvarea[i][j])/(dt+pT);
//				std::cout<<"p_mvarea[i][j]:"<<p_mvarea[i][j]<<"\n";
				int color=100*p_mvarea[i][j];
				for(int u=j*width/cnw;u<(j+1)*width/cnw;u++){
					for(int v=i*height/cnh;v<(i+1)*height/cnh;v++){
						Limg_view.at<cv::Vec3b>(v,u)[1]+=color;
					}
				}
			}
		}

//衝突物体を予測
//avept[i][j].x
		double obj_pstn;
		int prd_obj[cnh][cnw];
		int prd_line[cnw];
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				prd_obj[i][j]=0;
			}
		}
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				if(p_mvarea[i][j]!=0){	
					std::cout<<"avept["<<i<<"]["<<j<<"].x:"<<avept[i][j].x<<"\n";
//					std::cout<<"p_avez["<<i<<"]["<<j<<"]:"<<p_avez[i][j]<<"\n";
//					std::cout<<"p_avez["<<i<<"]["<<j<<"]-avez["<<i<<"]["<<j<<"]:"<<p_avez[i][j]-avez[i][j]<<"\n";
//					std::cout<<"t_limit["<<i<<"]["<<j<<"]:"<<avez[i][j]/dz*dt<<"\n";
//---
//					std::vector<cv::Point2i> mv_area;
/*					cv::Point2i temp;
					temp.x=j;
					temp.y=i;
					mv_area.push_back(temp);
					for(int k=0;k<mv_area.size();k++){
						if(p_mvarea[mv_area[k].y][mv_area[k].x]){
							opt.push_back(avept[mv_area[k].y][mv_area[k].x].x);
							p_mvarea[mv_area[k].y][mv_area[k].x]=0;
						}
						pp_mvarea[mv_area[k].y][mv_area[k].x]=1;
						
						for(int l=-1;l<=1;l+=2){
							int u=mv_area[k].x;
							int v=mv_area[k].y;
							if(pp_mvarea[v+l][u])
								continue;
							if(pp_mvarea[v][u+l])
								continue;
							std::cout<<"u,v"<<u<<","<<v<<"\n";
						
							if(std::abs(sp3d.sqr_p3d[v].line_p3d[u].z-sp3d.sqr_p3d[v+l].line_p3d[u].z)<0.05){
								temp.x=u;
								temp.y=v+l;
								mv_area.push_back(temp);
							}
							if(std::abs(sp3d.sqr_p3d[v].line_p3d[u].z-sp3d.sqr_p3d[v].line_p3d[u+l].z)<0.05){
								temp.x=u+l;
								temp.y=v;
								mv_area.push_back(temp);
							}
						}
						std::cout<<"k:"<<k<<"\n";	
					}		
					double ave_opt=0;
					for(int k=0;k<mv_area.size();k++){		
						ave_opt+=opt[k];						
					}
					ave_opt=ave_opt/(int)opt.size();
				
					
					for(int k=0;k<mv_area.size();k++){
//--
						double obj_pstn=ave_opt*avez[mv_area[k].y][mv_area[k].x]/dz;//
	//					std::cout<<"dz:"<<dz<<"\n";
						obj_pstn=((double)mv_area[k].x+0.5)*width/cnw+obj_pstn;
						std::cout<<"obj_pstn(prev,aftr):("<<((double)mv_area[k].x+0.5)*width/cnw<<","<<obj_pstn<<")\n";
						int prd_j=(int)(obj_pstn*cnw/width);
						std::cout<<"prd_j:"<<prd_j<<"\n";
						//移動するエリアは考慮しない
						if(0<=prd_j&&prd_j<cnw){
							prd_obj[mv_area[k].y][prd_j]=1;
							//移動後z
							if(sp3d.sqr_p3d[mv_area[k].y].line_p3d[prd_j].z >avez[mv_area[k].y][mv_area[k].x])
								sp3d.sqr_p3d[mv_area[k].y].line_p3d[prd_j].z =avez[mv_area[k].y][mv_area[k].x];
						}
	//						(pT*pprd_obj[i][j]+dt*prd_obj[i][j])/(pT+dt);

						int color=100;
						for(int u=mv_area[k].x*width/cnw;u<(mv_area[k].x+1)*width/cnw;u++){
							for(int v=mv_area[k].y*height/cnh;v<(mv_area[k].y+1)*height/cnh;v++){
								Limg_view.at<cv::Vec3b>(v,u)[0]+=color;
							}
						}
					}*/
					double obj_pstn=avept[i][j].x*avez[i][j]/dz;//
//					std::cout<<"dz:"<<dz<<"\n";
					obj_pstn=((double)j+0.5)*width/cnw+obj_pstn;
//					obj_pstn=((double)j+0.5)*width/cnw+obj_pstn;
//					std::cout<<"obj_pstn(prev,aftr):("<<((double)j+0.5)*width/cnw<<","<<obj_pstn<<")\n";
					int prd_j=(int)(obj_pstn*cnw/width);
					std::cout<<"j,prd_j,avez,areaz:"<<j<<","<<prd_j<<","<<avez[i][j]<<","<<sp3d.sqr_p3d[i].line_p3d[j].z<<"\n";
					//移動するエリアは考慮しない
					if(0<=prd_j&&prd_j<cnw){
						prd_obj[i][prd_j]=1;
						//移動後z
						if(sp3d.sqr_p3d[i].line_p3d[prd_j].z >avez[i][j]){
							sp3d.sqr_p3d[i].line_p3d[prd_j].z =avez[i][j];
//							std::cout<<"sp3d.sqr_p3d[i].line_p3d[prd_j].z =avez[i][j]:"<<sp3d.sqr_p3d[i].line_p3d[prd_j].z<<","<<avez[i][j]<<"\n";
						}
					}
//						(pT*pprd_obj[i][j]+dt*prd_obj[i][j])/(pT+dt);

/*					int color=100;
					for(int u=j*width/cnw;u<(j+1)*width/cnw;u++){
						for(int v=i*height/cnh;v<(i+1)*height/cnh;v++){
							Limg_view.at<cv::Vec3b>(v,u)[0]+=color;
						}
					}
*/
				}
			}
		}
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				//移動後エリアを追加
				if(prd_obj[i][j]!=0){
					double Tp=dt*5;
//					prd_obj[i][j]=(pT*pprd_obj[i][j]+dt*prd_obj[i][j])/(pT+dt);
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
						if(prd_obj[i][j]==0){
							int mrgn=3;//(int)(f*0.5/sp3d.sqr_p3d[i].line_p3d[j].z/width*cnh)*2+1;///3;
//							std::cout<<"mrgn:"<<mrgn<<"\n";
							bool cnf=false;
							for(int k=-mrgn/2;k<mrgn/2;k++){
								if(j+k<0||j+k>cnh-1){
//									cnf=true;
									continue;
								}
								else{
									if(p_mvarea[i][j+k]!=0){
										cnf=true;
										break;
									}
								}
							}
							if(cnf)
								continue;
							
						}
						if(p_mvarea[i][j]==0){
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
			if(all_count==0||min_z<0.5||max_z>5)//add
				min_z=0.5;
			else if((float)nan_count/all_count>=0.70)
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
		const double rw=0.276+0.2;
		double w_pix=f*rw/min_line_z;
		int space_minsize=w_pix/(width/cnw)+1;
		std::cout<<"space_minsize:"<<space_minsize<<"\n";
		std::vector<int> space_begin;
		std::vector<int> space_end;
		std::vector<int> space_size;
		int space_temp=0;
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
				for(int k=0;k<space_minsize;k++)
					z_aveline+=line_z[space_begin[i]+j+k];
				z_aveline=z_aveline/space_minsize;
				if(z_target<z_aveline){
					z_target=z_aveline;
					target_num=space_begin[i]+j+space_minsize/2;
				}
				else if(z_target==z_aveline
					||std::abs(z_target-z_aveline)<z_target*0.1 ){
					if(std::abs(target_num-cnw/2)>std::abs(space_begin[i]+j+space_minsize/2-cnw/2))
						target_num=space_begin[i]+j+space_minsize/2;
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


//		std::cout<<"target_num:"<<target_num<<"\n";
//		std::cout<<"z_target:"<<z_target<<"\n";
		target_point.x=width/cnw*target_num;
//		target_point.x=(pT*ptarget_point.x+dt*target_point.x)/(pT+dt);
		target_point.y=height/2;
		cv::circle(Limg_view, target_point, 4, cv::Scalar(200,200,0),-1, CV_AA);
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				p_pmvarea[i][j]=p_mvarea[i][j];
				p_avez[i][j]=avez[i][j];
//				p_avez[i][j]=avept[i][j].x*sp3d.sqr_p3d[i].line_p3d[j].z;
						pprd_obj[i][j]=prd_obj[i][j];
			}
		}
		ptarget_point=target_point;
	}

