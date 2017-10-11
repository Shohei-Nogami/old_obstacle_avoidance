#include"img_prc_cls.h"

	void ImageProcesser::depth_callback(const sensor_msgs::ImageConstPtr& msg)
	{
	    try{
		depthimg= cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	    }
	    catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.",
		msg->encoding.c_str());
		return ;
	    }
	}
//set depth cvbridge image
	void ImageProcesser::setdepth(void){
	  depth_queue.callOne(ros::WallDuration(1));
	}
//set depth mat image
	void ImageProcesser::setdepth_img(void){
		depth_img=depthimg->image.clone();
	}
	//publish depth image
	void ImageProcesser::pub_depthimg(void){
		pub_dpt.publish(depthimg->toImageMsg());
	}
	void ImageProcesser::approx_depth_img(void){
/*
その1 X軸で線形近似
depth1|nan1|nan2|depth2

nan[i]=depth1+(depth2-depth1)/n * i
n:nanの数
value nan nan value
        +1   +1       

for height i
for width  j  depth1=0 depth2=0 count=0
  depth=img(j,i)
  if depth != nan && img(j+1,i) == nan 
    depth1 = depth
    count++
  if   depth== nan && img(j+1,i) == nan
    count++  
  if depth==nan && img(j+1,i) != nan
    depth2= img(j+1,i)
    if depth1==0
      for k=0 count!=k k++
        img(j-k,i)=depth2
    else
      for k=0 count!=k k++
        img(j-k,i)=depth2-(depth2-depth1)/(count+1)*(k+1)
  if j==(width-1) && img(j,i)==nan//配列の終端
      for k=0 count!=k k++
        img(j-k,i)=depth1
*/
//.at(y,x)
	    for(int i=0;i<height;i++){
	      for(int j=0,depth=0,depth1=0,depth2=0,count=0;j<width-1;j++){
	        depth=depth_img.at(i,j);
	        //|val|nan|のとき
	        if(!std::isnan(depth) && std::isnan(depth_img.at(i,j+1))){
	          depth1=depth;
	          count++;
	        }
	        
	        if(std::isnan(depth)){
	          //|nan|nan|の区間
	          if(std::isnan(depth_img.at(i,j+1)))
	            count++;
	          //|nan|val|のとき
	          else{
	            depth2=depth_img.at(i,j+1);
	            //左端がnanのとき
	            if(depth1==0){
	              for(int k=0;k<count;k++)
	                depth_img.at(i,j-k)=depth2;
	            }
	            
	            else{
	              for(int k=0;k<count;k++)
	                depth_img.at(i,j-k)=depth2-(depth2-depth1)/(count+1)*(k+1);
	                
	            }
	          }
	        }
	        //右端がnanのとき
	        if(j==(width-1)-1 &&std::isnan(depth_img.at(i,j+1))){
	          for(int k=0;k<count;k++)
	            depth_img.at(i,j-k)=depth1;
	          
	        }
	      }
	    }
	}

