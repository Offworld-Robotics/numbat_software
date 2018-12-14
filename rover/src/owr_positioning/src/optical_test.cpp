#define _USE_MATH_DEFINES
#include <fstream>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <utility>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ecl/threads.hpp>
#include "optical_localization.h"

double pixels_per_metre = 1;

// correct image for distortion, edge detection and moment analysis
// geometry_msgs::Twist process_ground_truth(const cv::Mat &current, const cv::Mat &prev) {
//     return this_twist;
// }

// optical flow -> RANSAC
geometry_msgs::Twist process_baseline(const cv::Mat &current, const cv::Mat &prev) {
    cv::Mat affineTransform = estimateRigidTransform(prev, current, false);
    cv::Mat translation_delta = affineTransform.col(2);
    double rotation_delta = atan2(affineTransform.at<double>(1,1), affineTransform.at<double>(0,1)) - M_PI/2.0;
    double time_scale = 0;
    time_scale = 1.0/30;
    geometry_msgs::Twist this_twist;
    this_twist.linear.x = translation_delta.at<double>(0);
    this_twist.linear.y = translation_delta.at<double>(1);
    this_twist.angular.z = rotation_delta;
    return this_twist;
}

geometry_msgs::Twist process_test1(const cv::Mat &current, const cv::Mat &prev) {
    std::vector<cv::Point2f> corners;
    double quality_level = 0.04;
    int max_corners = 100;
    double min_distance = 0.01;
    int block_size = 3;
    bool use_harris_detector = false;
    double k = 0.04;
    cv::Mat prev_copy = prev.clone();
    // shi tomasi corners
    cv::goodFeaturesToTrack(prev_copy, corners, max_corners, quality_level, min_distance, cv::Mat(), 
        block_size, use_harris_detector, k);
    std::vector<cv::Point2f> curr_corners;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Size window_size(3,3);
    int pyramid_level = 5;
    cv::calcOpticalFlowPyrLK(prev_copy, current, corners, curr_corners, status, err,
        window_size, pyramid_level);
    std::vector<cv::Point2f> displacements;
    std::vector<float> angles;
    for (std::size_t i = 0; i < corners.size(); i++) {
        if (status[i] == 1) {
            cv::Point2f disp = curr_corners[i] - corners[i];
            displacements.push_back(disp);
            angles.push_back(atan2(disp.y, disp.x));
        }
    }
    std::ofstream angles_file("angles.txt");
    for (std::vector<float>::iterator it = angles.begin();  it != angles.end(); ++it) {
        angles_file << *it << "\n";
    }
    geometry_msgs::Twist this_twist;
    return this_twist;
}

void update_pose(geometry_msgs::Pose2D &pose, const geometry_msgs::Twist &vel) {
    double delta_t = 1.0/30;
    pose.x += vel.linear.x * delta_t;
    pose.y += vel.linear.y * delta_t;
    pose.theta += vel.angular.z * delta_t;
}

void write_data(std::ofstream &f, const geometry_msgs::Twist &vel) {
    f << vel.linear.x << " ";
    f << vel.linear.y << " ";
    f << vel.angular.z << "\n";
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "optical_flow");
    // VideoCapture ground_cap = cv::capture("ground_truth.mp4");
    // if (!ground.isOpened()) {
    //     ROS_INFO("Failed to open ground video");
    //     return -1;
    // }
    cv::VideoCapture data_cap("v2.mp4");
    if (!data_cap.isOpened()) {
        ROS_INFO("Failed to open data video");
        return -1;
    }
    cv::Mat ground_frame[2];
    cv::Mat data_frame[2];
    // ground_cap >> gound_frame[0];
    // cv::cvtColor(ground_frame[0], ground_frame[0], cv::COLOR_BGR2GRAY); // convert to grayscale
   
    data_cap >> data_frame[0];
    cv::cvtColor(data_frame[0], data_frame[0], cv::COLOR_BGR2GRAY);
    geometry_msgs::Twist baseline_vel;
    geometry_msgs::Pose2D baseline_pose;
    std::ofstream ground_truth("ground_truth.txt");
    std::ofstream baseline("baseline.txt");
    int i = 0;
    while (1) {
        // ground_cap >> ground_frame[1]; // get next frame
        data_cap >> data_frame[1];
        if (data_frame[1].empty()) {
            ROS_INFO("done processing, exiting");
            break;
        }
        // cv::cvtColor(ground_frame[1], ground_frame[1], cv::COLOR_BGR2GRAY); // convert to grayscale
        cv::cvtColor(data_frame[1], data_frame[1], cv::COLOR_BGR2GRAY);
        if(!data_frame[0].empty()) {
            // baseline_vel = process_baseline(data_frame[1], data_frame[0]);
            process_test1(data_frame[1], data_frame[0]);
            write_data(baseline, baseline_vel);
            if (i==80)
                break;
        }
        i += 1;
        // ground_frame[0] = ground_frame[1];
        data_frame[0] = data_frame[1];
        write_data(baseline, baseline_vel);
    }
    return 0;
}
