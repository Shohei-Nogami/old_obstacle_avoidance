#include"img_prc_cls.h"

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
		bool ismvobj[cnh][cnw];
		bool ismvline[cnw]={false};
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
				ismvobj[i][j]=false;
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
//		double ddt=dt/0.045;
		for(int i=0;i<cnh;i++){
			for(int j=0;j<cnw;j++){
				th_dsp=1*std::abs(dyaw)/0.01*std::abs(j-cnw/2+0.5);
				th_mv=2.0;///ddt;
				if(std::abs(dyaw)>0.01)
					th_mv=th_mv*std::abs(dyaw)/0.01;
				if(std::isnan(avept[i][j].x)||avept[i][j].y*avez[i][j]/f+0.23<0.15||std::abs(avept[i][j].x)<th_mv/avez[i][j]||th_mv<dsppt[i][j].x)
//				if(std::isnan(avesize[i][j]))//||dspsize[i][j]>th_dsp||avesize[i][j]<1||avesize[i][j]<th_dsp)
					continue;
				else{
					ismvobj[i][j]=true;
//					std::cout<<"ddt:"<<ddt<<"\n";
					std::cout<<"th_dsp:"<<th_dsp<<"\n";
					std::cout<<"th_mv:"<<th_mv<<"\n";
					std::cout<<"avept[i][j]:"<<avept[i][j]<<"\n";
					std::cout<<"avesize[i][j]:"<<avesize[i][j]<<"\n";
					std::cout<<"dspsize[i][j]:"<<dspsize[i][j]<<"\n";
					std::cout<<"avez[i][j]:"<<avez[i][j]<<"\n";
/*					p_mv=avesize[i][j];///th_dsp;
					if(th_dsp>1)
						p_mv/=th_dsp;
					if(p_mv>1)
						p_mv=1;
					std::cout<<"p_mv:"<<p_mv<<"\n";
*/
					p_mv=1;
					int color=100*p_mv;
					for(int u=j*width/cnw;u<(j+1)*width/cnw;u++){
						for(int v=i*height/cnh;v<(i+1)*height/cnh;v++){
							Limg_view.at<cv::Vec3b>(v,u)[1]+=color;
						}
					}
				}
			}
		}
		
//	convert ismvobj->ismvline	
		for(int j=0;j<cnw;j++){
			for(int i=0;i<cnh;i++){
				if(ismvobj[i][j]==true){
					ismvline[j]=true;
					break;
				}				
			}
		}
//	find spaces		
		int space_begin;
		int space_end;
		int space_size=0;
		int space_temp=0;
/*		for(int j=0;j<cnw;j++){
			std::cout<<ismvline[j];
		}
		std::cout<<"\n";
*/
		for(int j=0;j<cnw;j++){
			if(!ismvline[j])
				space_temp++;
			else{
				if(space_size<=space_temp){
					space_size=space_temp;
					space_end=j;
					space_begin=space_end-space_size;
				}
				space_temp=0;
			}
			if(j==cnw-1){
				if(space_size<=space_temp){
					space_size=space_temp;
					space_end=j;
					space_begin=space_end-space_size;
				}			
			}
		}
	//	std::cout<<"space:"<<space_size<<"["<<space_begin<<","<<space_end<<"]\n";
//culculate target point
		cv::Point2i target_point;
		target_point.x=(space_begin+space_end+1)/2*width/cnw+width/cnw*0.5;
		target_point.y=height/2;
		cv::circle(Limg_view, target_point, 4, cv::Scalar(200,200,0),-1, CV_AA);
	}

