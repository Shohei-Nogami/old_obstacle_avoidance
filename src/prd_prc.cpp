#include"img_prc_cls.h"

	void ImageProcesser::prd_prcess(void){
		//方針
		//分割領域のオプティカルフローの大きさの平均or平均と分散を算出
		//オプティカルフローがない領域なども考慮に入れること
		//領域全体の平均と分散から、平均からより離れている領域に番号付け（ソート）を行い、移動物体領域を検出
		//デバック::より移動物体として認識されている領域に色を加える
		
		//分割画像の各特徴点の平均と分散(x,y,size)
		cv::Point2d avept[cn][cn];	//sum->ave
		double avesize[cn][cn];		//sum->ave
		cv::Point2d dsppt[cn][cn];	//sum->dsp
		double dspsize[cn][cn];		//sum->dsp
		double p_mvarea[cn][cn];
		//initialize

		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				avept[i][j].x=0;
				avept[i][j].y=0;
				avesize[i][j]=0;
				dsppt[i][j].x=0;
				dsppt[i][j].y=0;
				dspsize[i][j]=0;
			}
		}
		//calculate sum
		for(int k=0;k<points.size();k++){
			for(int j=0;j<cn;j++){
				if((int)(j*width/cn) < (int)points[k].x && (int)points[k].x < (int)((j+1)*width/cn)){
					for(int i=0;i<cn;i++){
						if((int)(i*height/cn)<(int)points[k].y&&(int)points[k].y<(int)((i+1)*height/cn)){
							cpt[i][j].push_back(points[k]);
							cnpt[i][j].push_back(newpoints[k]-jnewpoints[k]+points[k]);
							avept[i][j].x+=newpoints[k].x-jnewpoints[k].x;
							avept[i][j].y+=newpoints[k].y-jnewpoints[k].y;
							avesize[i][j]+=sqrt(pow(newpoints[k].x-jnewpoints[k].x,2.0)
								+pow(newpoints[k].y-jnewpoints[k].y,2.0));
						}
					}
				}
			}
		}
		//calculate average
		for(int j=0;j<cn;j++){
			for(int i=0;i<cn;i++){
				avept[i][j].x=avept[i][j].x/(int)cpt[i][j].size();
				avept[i][j].y=avept[i][j].y/(int)cpt[i][j].size();
				avesize[i][j]=avesize[i][j]/(int)cpt[i][j].size();
			}
		}
		//calculate dispersion
		for(int j=0;j<cn;j++){
			for(int i=0;i<cn;i++){
				for(int k=0;k<cpt[i][j].size();k++){
					dsppt[i][j].x+=(cpt[i][j][k].x-avept[i][j].x)*(cpt[i][j][k].x-avept[i][j].x);
					dsppt[i][j].y+=(cpt[i][j][k].y-avept[i][j].y)*(cpt[i][j][k].y-avept[i][j].y);
					dspsize[i][j]+=pow( sqrt(pow(newpoints[k].x-jnewpoints[k].x,2.0)
							+pow(newpoints[k].y-jnewpoints[k].y,2.0)) -avesize[i][j],2.0);				
				}
				dsppt[i][j].x=sqrt( dsppt[i][j].x/(int)cpt[i][j].size() );				
				dsppt[i][j].y=sqrt( dsppt[i][j].y/(int)cpt[i][j].size() );
				dspsize[i][j]=sqrt( dspsize[i][j]/(int)cpt[i][j].size() );
			}
		}
		double ave_area=0;
		double dsp_area=0;
		//calculate area average
		int nan_count=0;
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				if(std::isnan(avesize[i][j]))
					nan_count++;
				else
					ave_area+=avesize[i][j];
			}
		}
		ave_area=ave_area/(cn*cn-nan_count);
		//calculate area dispersion (standard deviation)
		nan_count=0;
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				if(std::isnan(avesize[i][j]))
					nan_count++;
				else
					dsp_area+=pow(avesize[i][j]-ave_area,2.0);
			}
		}
		dsp_area=sqrt(dsp_area/(cn*cn-nan_count));
		std::cout<<"avearea:"<<ave_area<<"\n";
		//culculate probability of presence of moving objects
		double pT=dt*10;
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				if(!std::isnan(avesize[i][j])){
					p_mvarea[i][j]=1-exp( -pow(avesize[i][j]-ave_area,2.0)/(2*pow(dsp_area,2.0)) );
					if(!std::isnan(p_pmvarea[i][j]))
						p_mvarea[i][j]=(pT*p_pmvarea[i][j]+dt*p_mvarea[i][j])/(pT+dt);
//						std::cout<<"p_mvarea[i][j]:"<<p_mvarea[i][j]<<"\n";
				}
			}
		}
//		std::cout<<"p_mvarea:"<<p_mvarea[cn/2][cn/2]<<"\n";
			//j*width/cn+itk->pt.x
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				if(std::isnan(avesize[i][j]))
					continue;
				int color=200*p_mvarea[i][j];
				for(int u=j*width/cn;u<(j+1)*width/cn;u++){
					for(int v=i*height/cn;v<(i+1)*height/cn;v++){
						Limg_view.at<cv::Vec3b>(v,u)[1]+=color;
					}
				}
			}
		}
		for(int i=0;i<cn;i++){
			for(int j=0;j<cn;j++){
				p_pmvarea[i][j]=p_mvarea[i][j];
			}
		}
	}

