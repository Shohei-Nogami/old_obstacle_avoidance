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
//		double avez[cnh][cnw];
		double p_mvarea[cnh][cnw];
		double ismvobj[cnh][cnw];
		double ismvline[cnw]={0};
		std::vector<cv::Point2i> obj_begin;
		std::vector<cv::Point2i> obj_end;
		std::vector<double> obj_opt;
		//initialize

		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				avept[i][j].x=0;
				avept[i][j].y=0;
				avesize[i][j]=0;
//				avez[i][j]=0;
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
//							avez[i][j]+=z[k];
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
//				avez[i][j]=avez[i][j]/(int)cpt[i][j].size();
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
				th_dsp=1*std::abs(dyaw)/0.01*std::abs(j-cnw/2+0.5);
				th_mv=2.0;///ddt;
				if(std::abs(dyaw)>0.01)
					th_mv=th_mv*std::abs(dyaw)/0.01;
//				if(i>=9)
//					std::cout<<"sp3d.sqr_p3d[i"<<i<<"].line_p3d[j"<<j<<"].y:"<<sp3d.sqr_p3d[i].line_p3d[j].y<<"\n";
				if(sp3d.sqr_p3d[i].line_p3d[j].y+0.23<=0||sp3d.sqr_p3d[i].line_p3d[j].y+0.23>1.0
					||std::isnan(avept[i][j].x)
					||std::abs(avept[i][j].x)<th_mv/sp3d.sqr_p3d[i].line_p3d[j].z
					||th_mv<dsppt[i][j].x){
//				if(std::isnan(avesize[i][j]))//||dspsize[i][j]>th_dsp||avesize[i][j]<1||avesize[i][j]<th_dsp)
					p_mvarea[i][j]=0;
				}
				else{
//					std::cout<<"ddt:"<<ddt<<"\n";
//					std::cout<<"th_dsp:"<<th_dsp<<"\n";
//					std::cout<<"th_mv:"<<th_mv<<"\n";
//					std::cout<<"avept[i][j]:"<<avept[i][j]<<"\n";
//					std::cout<<"avesize[i][j]:"<<avesize[i][j]<<"\n";
//					std::cout<<"dspsize[i][j]:"<<dspsize[i][j]<<"\n";
//					std::cout<<"avez[i][j]:"<<avez[i][j]<<"\n";
/*					p_mv=avesize[i][j];///th_dsp;
					if(th_dsp>1)
						p_mv/=th_dsp;
					if(p_mv>1)
						p_mv=1;
					std::cout<<"p_mv:"<<p_mv<<"\n";
*/
					p_mvarea[i][j]=1;
//zの大きさによる重み付け
//z<=min_z==1m -> p_mvarea=1 (危険領域)
//min_z,max_zで線形近似
//min_z < z < max_z (警戒領域)
//max_z==5m < z	-> p_mvarea=0 (安全領域)
					double max_z=5;
					double min_z=1;
					if(sp3d.sqr_p3d[i].line_p3d[j].z<=min_z)
						p_mvarea[i][j]=min_z;
					else if(sp3d.sqr_p3d[i].line_p3d[j].z>=max_z)
						p_mvarea[i][j]=0;
					else
						p_mvarea[i][j]=-min_z/(max_z-min_z) * sp3d.sqr_p3d[i].line_p3d[j].z + (max_z)/(max_z-min_z);
				}
		
//				p_mvarea[i][j]=(pT*p_pmvarea[i][j]+dt*p_mvarea[i][j])/(pT+dt);
				if(std::isnan(p_mvarea[i][j])||p_mvarea[i][j]<0)
					p_mvarea[i][j]=0;
//				std::cout<<"p_mvarea[i][j]:"<<p_mvarea[i][j]<<"\n";
				int color=200*p_mvarea[i][j];
				for(int u=j*width/cnw;u<(j+1)*width/cnw;u++){
					for(int v=i*height/cnh;v<(i+1)*height/cnh;v++){
						Limg_view.at<cv::Vec3b>(v,u)[1]+=color;
					}
				}
			}
		}

//	convert ismvobj->ismvline
//	ismvline : ave_mvobj
		//移動物体領域合成用
		cv::Point2i area_begin;
		cv::Point2i area_end;
		double sum_pmvline[cnw]={0};
		double pmvline[cnw]={0};
		double line_z[cnw]={0};
		int z_count=0;
		for(int j=0;j<cnw;j++){
			double min_z=5;
			int nan_count=0;
			for(int i=0;i<cnh;i++){
				if(!std::isnan(sp3d.sqr_p3d[i].line_p3d[j].y)){
					if(sp3d.sqr_p3d[i].line_p3d[j].y+0.23>0&&sp3d.sqr_p3d[i].line_p3d[j].y+0.23<1.0){
	//					line_z[j]+=sp3d.sqr_p3d[i].line_p3d[j].z;
	//					z_count++;
						if(min_z>sp3d.sqr_p3d[i].line_p3d[j].z)
							min_z=sp3d.sqr_p3d[i].line_p3d[j].z;
					}
				}
				else
					nan_count++;
			}
			
			if(nan_count==cnh)
				min_z=0.5;	
//			if(min_z==5)
//				min_z=0.5;
/*			if(z_count!=0)
				line_z[j]=line_z[j]/z_count;
			else
				line_z[j]=0;
*/
			line_z[j]=min_z;
			for(int i=0;i<cnh;i++){
				if(p_mvarea[i][j]==1){
					pmvline[j]=1;
					break;
				}
				else{
					sum_pmvline[j]+=p_mvarea[i][j];
				}
			}
			if(pmvline[j]!=1)
				pmvline[j]=sum_pmvline[j]/cnh;
		}
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

					bool flag=false;
					for(int h=area_begin.y;h<area_end.y+1;h++){
						for(int w=area_begin.x;w<area_end.x+1;w++){
//--
							double obj_pstn;
							if(!std::isnan(avez[h][w])){
								obj_pstn=ave_opt*avez[h][w]/dz;//
							}
							else{
								obj_pstn=ave_opt*sp3d.sqr_p3d[h].line_p3d[w].z/dz;
							}
		//					std::cout<<"dz:"<<dz<<"\n";
							obj_pstn=((double)w+0.5)*width/cnw+obj_pstn;
//							std::cout<<"obj_pstn(prev,aftr):("<<((double)w+0.5)*width/cnw<<","<<obj_pstn<<")\n";

							int prd_j=(int)(obj_pstn*cnw/width);
//							if(pp_mvarea[h][w]==1)
//								std::cout<<"pp_mvarea[h][w]==1\n";
							std::cout<<"obj_pstn(prev,aftr):("<<w<<","<<prd_j<<")\n";
//							std::cout<<"prd_j:"<<prd_j<<"\n";
							//移動するエリアは考慮しない
							if(0<=prd_j&&prd_j<cnw){
								if(flag==false){
									obj_begin.push_back(area_begin);
									obj_end.push_back(area_end);
									obj_opt.push_back(ave_opt);
								}
								prd_obj[h][prd_j]=1;
								ppp_mvarea[h][w]=1;
								//移動後z
								if(!std::isnan(avez[h][w])){	
									if(sp3d.sqr_p3d[h].line_p3d[prd_j].z >avez[h][w])
										sp3d.sqr_p3d[h].line_p3d[prd_j].z =avez[h][w];
								}
								else{
									if(sp3d.sqr_p3d[h].line_p3d[prd_j].z >sp3d.sqr_p3d[h].line_p3d[w].z)
										sp3d.sqr_p3d[h].line_p3d[prd_j].z =sp3d.sqr_p3d[h].line_p3d[w].z;
								}
							}
		//						(pT*pprd_obj[i][j]+dt*prd_obj[i][j])/(pT+dt);
							int color=100;
							for(int u=w*width/cnw;u<(w+1)*width/cnw;u++){
								for(int v=h*height/cnh;v<(h+1)*height/cnh;v++){
//									Limg_view.at<cv::Vec3b>(v,u)[0]+=color;
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
		int max_size_num=0;
		int max_size=0;
		double max_size_opt=0;
		for(int i=0;i<obj_begin.size();i++){
			int size=(obj_end[i].x-obj_begin[i].x)*(obj_end[i].y-obj_begin[i].y);
			if(max_size_num==0){
				max_size_num=i;
				max_size=size;
				max_size_opt=obj_opt[i];
			}
			else{
				if(max_size<size){
					max_size_num=i;
					max_size=size;
					max_size_opt=obj_opt[i];
				}
			}
		}
		

//	find spaces
		const double rw=0.276;
		double w_pix=f*rw/min_line_z;
		int space_minsize=w_pix/(width/cnw)+1;
		//img_prc_cls.h reserve(cnw)
		std::vector<int> space_begin;
		std::vector<int> space_end;
		std::vector<int> space_size;
		int space_temp=0;
/*		for(int j=0;j<cnw;j++){
			std::cout<<ismvline[j];
		}
		std::cout<<"\n";
*/
//VFH
		for(int j=0;j<cnw;j++){
			if(line_z[j]>1&&!std::isinf(line_z[j])&&!std::isnan(line_z[j]))
				space_temp++;
			else{
				if(space_minsize<=space_temp){
					space_size.push_back(space_temp);
					space_end.push_back(j);
					space_begin.push_back(j-space_temp);
				}
				space_temp=0;
			}
			if(j==cnw-1){
				if(space_minsize<=space_temp){
					space_size.push_back(space_temp);
					space_end.push_back(j);
					space_begin.push_back(j-space_temp);
				}
			}
		}

		double z_max=0.5;
		double z_aveline=0;//sum->ave
		int target_num=cnw/2;
		for(int i=0;i<space_size.size();i++){
			//space num
			for(int j=0;j<space_size[i]-(space_minsize-1);j++){
				for(int k=0;k<space_minsize;k++)
					z_aveline+=line_z[space_begin[i]+j+k];
				z_aveline=z_aveline/space_minsize;
				if(z_max<z_aveline){
					z_max=z_aveline;
					target_num=space_begin[i]+j+space_minsize/2;
				}
				else if(z_max==z_aveline){
					if(std::abs(target_num-cnw/2)>std::abs(space_begin[i]+j+space_minsize/2-cnw/2))
						target_num=space_begin[i]+j+space_minsize/2;
				}
				z_aveline=0;
			}
		}
		std::cout<<"z_max:"<<z_max<<"\n";

		if((int)obj_begin.size()!=0){
			if(max_size_opt>0){
				target_num=obj_begin[max_size_num].x-2;
			}
			else{
				target_num=obj_begin[max_size_num].x+2;				
			}
		}
		if(target_num>=cnw)
			target_num=cnw-1;
		if(target_num<0)
			target_num=0;
		
//		std::cout<<"space_size.size():"<<space_size.size()<<"\n";
//		std::cout<<"target_num:"<<target_num<<"\n";
//	std::cout<<"space:"<<space_size<<"["<<space_begin<<","<<space_end<<"]\n";
//culculate target point
//		cv::Point2i target_point;
		target_point.x=width/cnw*target_num;//+width/cnw*0.5;
//		std::cout<<"tp1"<<target_point.x<<"\n";
		target_point.x=(pT*ptarget_point.x+dt*target_point.x)/(pT+dt);
//		std::cout<<"tp2"<<target_point.x<<"\n";
		target_point.y=height/2;
		cv::circle(Limg_view, target_point, 4, cv::Scalar(200,200,0),-1, CV_AA);
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				p_pmvarea[i][j]=p_mvarea[i][j];
			}
		}
		ptarget_point=target_point;
	}

