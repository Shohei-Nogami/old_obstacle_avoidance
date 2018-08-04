#include"image_class.h"
#ifndef INCLUDE_DEPTH_IMAGE_PROCESS
#define INCLUDE_DEPTH_IMAGE_PROCESS

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
		image_transport::Publisher pub2;
	public:

		depth_image_class();
		virtual ~depth_image_class();
		void set_pre_filted_image(void);

		cv::Mat& get_filted_cur_image(void);
		cv::Mat& get_filted_pre_image(void);
		void define_variable(void);
		void image_callback(const sensor_msgs::ImageConstPtr& msg);
		void filtering_depth_image(void);
		void publish_debug_image(cv::Mat& temp_image);
		void publish_filted_cur_image(void);
};

#endif
