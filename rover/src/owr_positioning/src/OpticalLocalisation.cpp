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
#include <ros/console.h>
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
    pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/owr/optical_localization_twist", 10, true);
    seq = 0;


}

void OpticalLocalisation::run() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}


// TODO Calibration constant (currently pixels absolute instead of pixels per
//  metre)
void OpticalLocalisation::process_image(const sensor_msgs::Image::ConstPtr& image) {

    // Initialise the current frame's matrix (buffer)
    // And grab the time the image was taken.
    cv::Mat curr_gray_frame;
    ros::Time curr_frame_time = image.header.stamp;

    // Convert current image from RGB to Gray, store in current buffer.
    cv::cvtColor(cv_bridge::toCvCopy(image)->image, curr_gray_frame,
           cv::COLOR_RGB2GRAY);

    // If the previous image is empty, do nothing.
    if(!prev_gray.empty()) {
        // Create a vector with two elements, made of floats.
        cv::Mat flow(prev_gray.size(), CV_32FC2);
        // TODO: Fix magic numbers.
        cv::calcOpticalFlowFarneback(prev_gray, curr_gray_frame, flow, 0.5, 5, 5, 10, 5, 1.2, 0);

        // create reference matrix where each value is the index of itself.
        // Create a new Matrix (Mat) that is the size of the flow frame out
        // of 32 Float (2 channels).
        cv::Mat baseMat = cv::Mat(flow.size(), CV_32FC2);
        for (int i = 0; i < baseMat.rows; i++) {
            for (int j = 0; j < baseMat.cols; j++) {
                baseMat.at<cv::Vec2f>(i, j) = cv::Vec2f(i, j);
            }
        }

        // create final matrix
        // Final matrix is the reference matrix transformed by the
        // vector generated in the flow frame.
        cv::Mat toMat = baseMat + flow;

        // OpenCV stores all matrices internally as arrays with a header
        // denoting the terminating indices. This reshapes the matrix
        // representation to only hold 1 column and n rows.
        baseMat = baseMat.reshape(2, baseMat.rows * baseMat.cols);
        toMat = toMat.reshape(2, toMat.rows * toMat.cols);

        // Now that the data is in a format readable by
        // estimateAffinePartial2D, both are fed into it and a Matrix is
        // returned containing the transformation.
        cv::Mat at = cv::estimateAffinePartial2D(baseMat, toMat);

        geometry_msgs::TwistWithCovarianceStamped msg;
        // FIXME: Header time
        msg.header.stamp = ros::Time::now();
        msg.header.seq = seq;
        seq++;

        // Angular Velocity

        //

        // TODO: Angular z
        msg.twist.twist.linear.x = at.at<double>(0, 2);
        msg.twist.twist.linear.y = at.at<double>(1, 2);
        // msg.twist.twist.angular.z = ?
        pub.publish(msg);
    }

    // Update previous image value & time.
    prev_gray = curr_gray_frame;
    prev_time = curr_frame_time;
}

