/* 
	Class header for ImageConverter
	Converting ROS images to OpenCV for modification
*/

#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

static const std::string OPENCV_WINDOW = "Image Window";

class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public:
		ImageConverter();
		~ImageConverter();
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif

