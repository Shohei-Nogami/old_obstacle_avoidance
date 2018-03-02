#include"img_prc_cls.h"

ImageProcesser::vector_field_histgram(void){

//	double line_depth[width/ksize]; in header	
	double min_depth;
	double cur_depth;
//compressing depth data(culculation min depth)
	for(int w=0;w<width/ksize;w++){ 
		min_depth=100;
		for(int h=0;h<height/ksize;h++){
			cur_depth=depth_image.at<float>(h,w);
			if(!std::isnan(cur_depth)&&!std::isinf(cur_depth)&&min_depth>cur_depth){
				min_depth=cur_depth;
			}
		}
		if(min_depth==100)
			min_depth=0.5;
		line_depth[w]=min_depth;
	}
	
//culculating free area
	const double free_area_threshold=1.0;
//	double mask_line_depth[width/ksize]; in	header
	double right_depth=0;
	double left_depth=0;
	int space_num=0;
	for(int w=0;w<width/ksize;w++){
		if(line_depth[w]>free_area_threshold){
			mask_line_depth[w]=true;
			if(left_depth==0){
				left_depth=line_depth[w];
			}
			else{
				
			}
			space_num++;
		}
		else{
			mask_line_depth[w]=false;
			if(left_depth!=0){
				right_depth=line_depth[w-1];
				if(left_depth>right_depth)
					
				space_num=0;				
			}
		}
	}
	std::vector<double> space_size;
	 
}
