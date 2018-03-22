#include"ros/ros.h"
#include <ros/callback_queue.h>
//画像取得用
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
//opencv
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<sensor_msgs/image_encodings.h>

#include"image_class.h"

class depth_image_class : public image_class
{
  private:
		cv::Mat filted_cur_image;
		cv::Mat filted_pre_image;  
		
		const int ksize=5;
		const int median_param=5;
		int FILTER_TYPE;
		const int NOTHING=0;
		const int MEDIAN_FILTER=1;
		const int AVERAGE_FILTER=2;
		const int width=672;
		const int height=376;
	std::vector<float> depth_median;
public:

	depth_image_class();
	virtual ~depth_image_class();
	void set_pre_image(void);
	void define_variable(void);
	void image_callback(const sensor_msgs::ImageConstPtr& msg);
	void filtering_depth_image(void);

};

