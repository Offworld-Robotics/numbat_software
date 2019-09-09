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
#include "OpticalLocalisation.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "OpticalLocalisation");
    OpticalLocalisation ol;
    ol.run();
    return 0;
}

OpticalLocalisation::OpticalLocalisation() {
    sub = nh.subscribe("/OpticalLocalisation/image_raw", 1,
            &OpticalLocalisation::process_image, this);
    pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>
            ("/owr/OpticalLocalisation_twist", 10, true);
}

void OpticalLocalisation::run() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}

// FIXME:
// This is weird and broken will fix later
void OpticalLocalisation::process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int cam) {

    // Initialise the current frame's matrix (buffer)
    cv::Mat curr_gray_frame;
    // Convert current image from RGB to Gray, store in current buffer.
    cv::cvtColor(cv_bridge::toCvCopy(image)->image curr_gray_frame,
           cv::BGR2GRAY);

    // If the previous image is empty, do nothing.
    if(!prev_gray.empty()) {
        void calcOpticalFlowFarneback(InputArray prev, InputArray next,
                cv::Mat transformMatrix;
        //estimateAffinePartial2D(prev_gray, curr_gray_frame, transformMatrix,
        //        cv::RANSAC, )

    }

    // Update previous image value.
    prev_gray = curr_gray_frame;
}

void OpticalLocalisation::printTwist(const geometry_msgs::Twist &twist) {
    ROS_INFO("lin x: %f, lin y: %f, ang z: %f", twist.linear.x, twist.linear.y, twist.angular.z);
}

