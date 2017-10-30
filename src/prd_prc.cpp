#include"img_prc_cls.h"

  void ImageProcesser::prd_prcess(void){
//    pmpts.push_back(points[j]);
//    cmpts.push_back(newpoints[j]-jnewpoints[j]+points[j]);
//    pmz.push_back(z[j]);
//    cmz.push_back(nz[j]);
//    std::vector< std::vector<cv::Point2f> > area_mpt;
//	std::vector<cv::Point2f> onearea_mpt;
	if(!isPrevimage()||cmpts.empty()||cmpts.size()>=300)
		return ;
//	std::cout<<"in prd_prcess\n";
    std::vector<float> area_z;
    bool mpt_f[(int)cmpts.size()];
    const double th_len=0.10;//10cm
    const double space_th=0.30;//30cm
    std::vector<cv::Point2i> target_pt;//target points
    //各点の初期化
    for(int i=0;i<cmpts.size();i++){
      mpt_f[i]=false; 
//      cmpts[i]=cmpts[i]*z[i]/f;
//      pmpts[i]=pmpts[i]*z[i]/f;    
    }
//	std::cout<<"initialized\n";
    //各点をクラスタリング
    for(int i=0;i<cmpts.size();i++){
      double area_pz=0;
//	std::cout<<"01\n";
      if(mpt_f[i])
        continue;
//      area_mpt.push_back(cmpts[i]);
	cv::Point2f temp_area_mpt;
	temp_area_mpt.x=cmpts[i].x;
	temp_area_mpt.y=cmpts[i].y;
//	std::cout<<"1\n";
      onearea_mpt.push_back(temp_area_mpt);
//	std::cout<<"2\n";
      mpt_f[i]=true;
      for(int j=0;j<onearea_mpt.size();j++){
        for(int k=1;k<cmpts.size()&&ros::ok();k++){
          if(mpt_f[k])
            continue;
//          double length=sqrt((area_mpt[i][j].x-(cmpts[k].x-width/2)*z[i]/f)*(area_mpt[i][j].x-(cmpts[k].x-width/2)*z[i]/f)+
//                             (area_mpt[i][j].y-(cmpts[k].y-height/2)*z[i]/f)*(area_mpt[i][j].y-(cmpts[k].y-height/2)*z[i]/f));
          double length=sqrt( ((onearea_mpt[j].x-width/2)*z[j]/f-(cmpts[k].x-width/2)*z[k]/f)*((onearea_mpt[j].x-width/2)*z[j]/f-(cmpts[k].x-width/2)*z[k]/f)+
                             ((onearea_mpt[j].y-width/2)*z[j]/f-(cmpts[k].y-height/2)*z[k]/f)*((onearea_mpt[j].y-width/2)*z[j]/f-(cmpts[k].y-height/2)*z[k]/f) );
//	std::cout<<"i,j,k:"<<i<<","<<j<<","<<k<<"\n";
          if(length<=th_len){
            mpt_f[k]=true; 
            onearea_mpt.push_back(cmpts[i]);
            area_pz+=z[i];
          }
        }
      }
//	std::cout<<"onearea_mpt:"<<onearea_mpt.size()<<"\n";
//	std::cout<<"cmpts.size:"<<cmpts.size()<<"\n";
	area_mpt.push_back(onearea_mpt);
      area_z.push_back(area_pz/(int)area_mpt[i].size());
    }
	std::cout<<"area_mpt.size:"<<area_mpt.size()<<"\n";
//	std::cout<<"hoge\n";
    std::vector<cv::Point2f> tlpt,brpt;
    //同物体を矩形で囲みdebug
	//debug 
	if(area_mpt.size()>100)
		return ;
    for(int i=0;i<area_mpt.size();i++){
      //culculate rectagle points
      float x_max,y_max,x_min,y_min;
      x_max=area_mpt[i][0].x;
      x_min=area_mpt[i][0].x;
      y_max=area_mpt[i][0].y;
      y_min=area_mpt[i][0].y;
      for(int j=0;j<area_mpt[i].size();j++)
		std::cout<<"area_mpt[i][j]:"<<area_mpt[i][j]<<"\n";
      for(int j=1;j<area_mpt[i].size();j++){
        if(x_max<area_mpt[i][j].x)
          x_max=area_mpt[i][j].x;
        else if(x_min>area_mpt[i][j].x)
          x_min=area_mpt[i][j].x;
        if(y_max<area_mpt[i][j].y)
          y_max=area_mpt[i][j].y;
        else if(y_min>area_mpt[i][j].y)
          y_min=area_mpt[i][j].y;
      }
      tlpt.push_back(cv::Point(x_min,y_min));
      brpt.push_back(cv::Point(x_max,y_max));
      cv::rectangle(Limg_view,cv::Point((int)x_min,(int)y_min),cv::Point((int)x_max,(int)y_max),cv::Scalar(0,200,0),3,4);
	std::cout<<"rectangle:"<<cv::Point((int)x_min,(int)y_min)<<","<<cv::Point((int)x_max,(int)y_max)<<"\n";
    }
    //sort
    cv::Point2f temp;
    for(int i=0;i<tlpt.size();i++){
      for(int j=0;j<tlpt.size()-i-1;j++){
        if(tlpt[j].x>tlpt[j+1].x){
          temp=tlpt[j+1];
          tlpt[j+1]=tlpt[j];
          tlpt[j]=temp;
          temp=brpt[j+1];
          brpt[j+1]=brpt[j];
          brpt[j]=temp;
        } 
      }
    }
    //find space and culculate target points
    for(int i=0,j=0;i<tlpt.size()-1;i++){
      if(brpt[i].x<tlpt[i+1].x){
        double space_x=tlpt[i+1].x*area_z[i+1]/f-brpt[i].x*area_z[i]/f;
        if(space_x>=space_th){
          target_pt.push_back( cv::Point((int)((tlpt[i+1].x-brpt[i].x)/2+brpt[i].x),(int)(brpt[i].y+(tlpt[i+1].y-brpt[i].y))/2) );
          cv::circle(Limg_view,target_pt[j++], 4, cv::Scalar(200,0,0),-1, CV_AA);
        }
      }
    }
    if((int)target_pt.size()==0)
     return ;
    double onetarget_dx=target_pt[0].x-width/2;
    for(int i=1;i<target_pt.size();i++){
      if(std::abs(onetarget_dx)>std::abs(target_pt[i].x-width/2))
        onetarget_dx=target_pt[i].x-width/2;
    }
    cv::circle(Limg_view,cv::Point(onetarget_dx+width/2,height/2), 4, cv::Scalar(0,200,0),-1, CV_AA);
    
  }

