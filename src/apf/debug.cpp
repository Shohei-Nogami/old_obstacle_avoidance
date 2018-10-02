#include<apf.h>

void APF::set_pub_debug_images(void)
{
	int W=map_wi;
	int H=map_hi;
	
	for(int h0=0;h0<H;h0++){
		for(int w0=0;w0<W;w0++){
			//set potential
			float pot=pot_map.at<float>(h0,w0);//*std::abs(sum_pot);
			if(pot>0)
			{
				debug_image.at<cv::Vec3b>(h0,w0)[2] =pot*255;
			}
			else
			{
				debug_image.at<cv::Vec3b>(h0,w0)[0] =(-pot)*255;
				debug_image.at<cv::Vec3b>(h0,w0)[1] =(-pot)*255;
			}
			//set path
			}	
	}
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[0] =255;
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[1] =255;
	debug_image.at<cv::Vec3b>(xri.y,xri.x)[2] =255;
		
	publish_debug_image(debug_image);
	
}
void APF::publish_debug_image(const cv::Mat& temp_image){
	//std::cout<<"1\n";
	cv_bridge::CvImagePtr publish_cvimage(new cv_bridge::CvImage);
	//cv::Mat temp=new cv::Mat(temp_image.rows,temp_image.cols, CV_8UC3);
	//temp=temp_image.clone();
	//std::cout<<"12\n";
	//cv::Mat temp=temp.clone();
	publish_cvimage->encoding=sensor_msgs::image_encodings::BGR8;
	//std::cout<<"123\n";
	//std::cout<<"cv::Size(temp_image.rows,temp_image.cols)"<<cv::Size(temp_image.rows,temp_image.cols)<<"\n";
	
	//publish_cvimage->image = cv::Mat::zeros(cv::Size(temp_image.rows,temp_image.cols), CV_8UC3);
	//cv::resize(publish_cvimage->image,publish_cvimage->image,cv::Size(temp_image.rows,temp_image.cols));
	//publish_cvimage->image=temp.clone();
	publish_cvimage->image=temp_image.clone();
	//publish_cvimage->image=temp;
	//std::cout<<"1234\n";
	pub.publish(publish_cvimage->toImageMsg());

}

