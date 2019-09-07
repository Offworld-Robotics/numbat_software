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

    cv::Mat frame_gray;
    cv::cvtColor(cv_bridge::toCvCopy(image)->image, frame_gray, cv::COLOR_BGR2GRAY);

    if(!prev_gray.empty()) {
        cv::Mat affineTransform = estimateRigidTransform(prev_gray, frame_gray, false);
        if(!affineTransform.empty()) {
            cv::Mat translation_delta = affineTransform.col(2);
            double rotation_delta = atan2(affineTransform.at<double>(1,1), affineTransform.at<double>(0,1)) - M_PI/2.0;

            double time_scale = 0;
            ros::Time current_time = ros::Time::now();
            if(!prev_time.isZero()) {
                time_scale = (1.0 / ((current_time - prev_time).nsec / 1e9));
            }
            prev_time = current_time;

            geometry_msgs::Twist this_twist;
            this_twist.linear.x = translation_delta.at<double>(0) * time_scale / pixels_per_metre;
            this_twist.linear.y = translation_delta.at<double>(1) * time_scale / pixels_per_metre;
            this_twist.angular.z = rotation_delta * time_scale;

            align_axes(this_twist, cam);

            if(is_first) {
                is_first = false;
                most_recent_average = this_twist;
            } else {
                most_recent_average = average(most_recent, this_twist);
            }
            //printTwist(this_twist);
            most_recent = this_twist;

            publishTwist();
        }
    }
    prev_gray = frame_gray;
}

void OpticalLocalisation::printTwist(const geometry_msgs::Twist &twist) {
    ROS_INFO("lin x: %f, lin y: %f, ang z: %f", twist.linear.x, twist.linear.y, twist.angular.z);
}

