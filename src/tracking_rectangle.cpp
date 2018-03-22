#include"img_prc_cls.h"

	void ImageProcesser::set_previous_objects(void){
		if(objects.rect.size())
			previous_objects=objects;
		
	}
	void ImageProcesser::tracking_process(void){
		double distance_x,distance_y;
		double sum_width,sum_height;
//		std::vector<cv::Point2i> tracking_rect;//tracking value x:previous, y:current
		//initialize : clear vector
		tracking_rect.clear();
		for(int i=0;i<previous_objects.rect.size();i++){
			int similarity_number=-1;
			int similarity_value=width*height;
			for(int j=0;j<objects.rect.size();j++){
				distance_x = std::abs(previous_objects.rect[i].tl.x + previous_objects.rect[i].br.x
					- objects.rect[j].tl.x-objects.rect[j].br.x);
				distance_y = std::abs(previous_objects.rect[i].tl.y + previous_objects.rect[i].br.y
					- objects.rect[j].tl.y-objects.rect[j].br.y);
				sum_width = previous_objects.rect[i].br.x - previous_objects.rect[i].tl.x 
					+ objects.rect[j].br.x - objects.rect[j].tl.x;
				sum_height = previous_objects.rect[i].br.y - previous_objects.rect[i].tl.y 
					+ objects.rect[j].br.y - objects.rect[j].tl.y;
				std::cout<<"distance:x"<<distance_x<<", y"<<distance_y<<"\n";
				std::cout<<"sum:width"<<sum_width<<", height"<<sum_height<<"\n";
				
				if(distance_x < sum_width || distance_y < sum_height){//collision
					//Evaluation formula
//					std::cout<<"collision\n";
					int temp_similarity_value = std::abs(objects.rect[j].tl.x - previous_objects.rect[i].tl.x)
						+ std::abs(objects.rect[j].br.x - previous_objects.rect[i].br.x)
						+ std::abs(objects.rect[j].tl.y - previous_objects.rect[i].tl.y)
						+ std::abs(objects.rect[j].br.y - previous_objects.rect[i].br.y);
					if(similarity_value > temp_similarity_value){
						similarity_value = temp_similarity_value;
						similarity_number = j;
					}
				}
				cv::Point2i temp_tracking_rect;
				temp_tracking_rect.x=i;
				temp_tracking_rect.y=similarity_number;
				tracking_rect.push_back(temp_tracking_rect);
			}	
		}
	}
	void ImageProcesser::debug_tracking_process(void){
		std::cout<<"in_debug_tracking_process\n";
		std::cout<<"tracking_rect.size() is "<<tracking_rect.size()<<"\n";
		int color=200;
		for(int i=0;i<tracking_rect.size();i++){
			int tn=tracking_rect[i].y;//tracking_number(current)
			cv::Point2i tl;
			tl.x=objects.rect[tn].tl.x;
			tl.y=objects.rect[tn].tl.y;
			cv::Point2i br;
			br.x=objects.rect[tn].br.x;
			br.y=objects.rect[tn].br.y;

			for(int u=tl.x;u<br.x;u++){
				for(int v=tl.y;v<br.y;v++){
					detectd_image.at<cv::Vec3b>(v,u)[1]+=color;//BGR:[0],[1],[2]
				}
			}
		}
	}

