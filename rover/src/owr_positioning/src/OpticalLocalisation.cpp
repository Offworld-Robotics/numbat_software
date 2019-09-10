/*
* Date Started:
 * Date: ????-??-??
 * Updated: 2019/09/08
 * Original Author: ?????
 * Editors: Michael Lloyd, William Miles
 * ROS Node Name: OpticalLocalisation
 * ROS Package: owr_positioning
 * Purpose: Generates odometry via optical flow
 */

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <utility>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include "OpticalLocalisation.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "OpticalLocalisation");
    OpticalLocalisation ol;
    ol.run();
    return 0;
}

OpticalLocalisation::OpticalLocalisation() {
    sub = nh.subscribe("/testcam/image_raw", 1,
            &OpticalLocalisation::process_image, this);
    pub = nh.advertise<sensor_msgs::Image>("/oftest/image", 10, true);
}

void OpticalLocalisation::run() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}

void OpticalLocalisation::process_image(const sensor_msgs::Image::ConstPtr& image) {

    // Initialise the current frame's matrix (buffer)
    cv::Mat curr_gray_frame;

    // Convert current image from RGB to Gray, store in current buffer.
    cv::cvtColor(cv_bridge::toCvCopy(image)->image, curr_gray_frame,
           cv::COLOR_RGB2GRAY);

    // If the previous image is empty, do nothing.
    if(!prev_gray.empty()) {
        cv::Mat flow(prev_gray.size(), CV_32FC2); 
        cv::calcOpticalFlowFarneback(prev_gray, curr_gray_frame, flow, 0.5, 5, 5, 10, 5, 1.2, 0);

	// visualization
	cv::Mat flow_parts[2];
	cv::split(flow, flow_parts);
	cv::Mat magnitude, angle, magn_norm;
	cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
	cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
	angle *= ((1.f / 360.f) * (180.f / 255.f));

	//build hsv image
	cv::Mat _hsv[3], hsv, hsv8, bgr;
	_hsv[0] = angle;
	_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
	_hsv[2] = magn_norm;
	cv::merge(_hsv, 3, hsv);
	hsv.convertTo(hsv8, CV_8U, 255.0);
	cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR); 

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();
        pub.publish(msg);
    }

    // Update previous image value.
    prev_gray = curr_gray_frame;
}

