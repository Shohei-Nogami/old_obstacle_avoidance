#include"img_prc_cls.h"


//矢印描写用関数
	void ImageProcesser::cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color){
    int thickness=4;
    int lineType=8;
    int shift=0;
    cv::line(*img,pt1,pt2,color,thickness,lineType,shift);
    float vx = (float)(pt2.x - pt1.x);
    float vy = (float)(pt2.y - pt1.y);
    float v = sqrt( vx*vx + vy*vy );
    float ux = vx / v;
    float uy = vy / v;
		//矢印の幅の部分
    float w=5,h=10;
    cv::Point2i ptl,ptr;
    ptl.x = (int)((float)pt2.x - uy*w - ux*h);
    ptl.y = (int)((float)pt2.y + ux*w - uy*h);
    ptr.x = (int)((float)pt2.x + uy*w - ux*h);
    ptr.y = (int)((float)pt2.y - ux*w - uy*h);
		//矢印の先端を描画する
		//--例外処理(!(v==0))
		if(!(v==0)){
	    	cv::line(*img,pt2,ptl,color,thickness,lineType,shift);
	    	cv::line(*img,pt2,ptr,color,thickness,lineType,shift);
		}
	}
