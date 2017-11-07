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
		double sum_pmvline[cnw]={0};
		double pmvline[cnw]={0};
		double line_z[cnw]={0};
		int z_count=0;
		for(int j=0;j<cnw;j++){
			double min_z=5;
			for(int i=0;i<cnh;i++){
				if(!std::isnan(sp3d.sqr_p3d[i].line_p3d[j].y)
					&&sp3d.sqr_p3d[i].line_p3d[j].y+0.23>0&&sp3d.sqr_p3d[i].line_p3d[j].y+0.23<1.0){
//					line_z[j]+=sp3d.sqr_p3d[i].line_p3d[j].z;
//					z_count++;
					if(min_z>sp3d.sqr_p3d[i].line_p3d[j].z)
						min_z=sp3d.sqr_p3d[i].line_p3d[j].z;
				}
			}
			if(min_z==5)
				min_z=0.5;
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

//	find spaces
		int space_minsize=3;//  3area/1m 
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
//危険領域のみを考慮したスペース探査
/*		for(int j=0;j<cnw;j++){
			if(pmvline[j]==0)
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
//		std::cout<<"pmvline\n";
//		for(int i=0;i<cnw;i++) 
//			std::cout<<pmvline[i]<<"\n";
//警戒領域を考慮した目標点設定
		double p_min=1;
		double p_aveline=0;//sum->ave
		int target_num=cnw/2;
		for(int i=0;i<space_size.size();i++){
			//space num
			for(int j=0;j<space_size[i]-(space_minsize-1);j++){
				for(int k=0;k<space_minsize;k++)
					p_aveline+=pmvline[space_begin[i]+j+k];
				p_aveline=p_aveline/space_minsize;
				if(p_min>p_aveline){
					p_min=p_aveline;
					target_num=space_begin[i]+j+space_minsize/2;
				}
				else if(p_min==p_aveline){
					if(std::abs(target_num-cnw/2)>std::abs(space_begin[i]+j+space_minsize/2-cnw/2))
						target_num=space_begin[i]+j+space_minsize/2;
				}
				p_aveline=0;
			}
		}
*/
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

