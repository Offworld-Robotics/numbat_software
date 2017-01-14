#ifndef OPTICAL_LOCALIZATION_H
#define OPTICAL_LOCALIZATION_H

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>

class OpticalLocalization {
	private:
		ros::Subscriber sub;
		ros::Publisher pub;
		
		cv::Mat prev_gray;
		cv::Mat translation_current;
		double scale_x;
		double scale_y;
		double rotation;
		
		ros::Time prev_time;
		
		const unsigned int pixels_per_metre_traversed;
		const double linear_velocity_scale;
		const double rotation_velocity_scale;
		
		unsigned int seq;
		
		double angle_between(cv::Mat& a, cv::Mat& b);
		void image_callback(const sensor_msgs::Image::ConstPtr& image);
		
	public:
		OpticalLocalization(int argc, char *argv[]);
		void run();
	
};

#endif // OPTICAL_LOCALIZATION_H
