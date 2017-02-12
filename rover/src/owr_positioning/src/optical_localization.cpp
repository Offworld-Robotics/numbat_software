#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "optical_localization.h"

void optical_localization::process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int idx) {
    cv::Mat frame = cv_bridge::toCvCopy(image)->image;
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    
    if(!prev_gray[idx].empty()) {
        cv::Mat affineTransform = estimateRigidTransform(prev_gray[idx], frame_gray, false);
        if(!affineTransform.empty()) {
            cv::Mat translation_delta = affineTransform.col(2);
            double scale_x_delta = norm(affineTransform.col(0));
            double scale_y_delta = norm(affineTransform.col(1));
            double rotation_delta = atan2(affineTransform.at<double>(1,1), affineTransform.at<double>(0,1)) - M_PI/2.0;
            
            translation_current[idx] += translation_delta;
            scale_x[idx] *= scale_x_delta;
            scale_y[idx] *= scale_y_delta;
            rotation[idx] += rotation_delta;
            
            double time_scale = 0;
            ros::Time current_time = ros::Time::now();
            if(!prev_time[idx].isZero()) {
                time_scale = (1.0 / ((current_time - prev_time[idx]).nsec / 1e9));
            }
            prev_time[idx] = current_time;
            
            my_twist[idx].twist.twist.linear.x = translation_delta.at<double>(0) * time_scale / pixels_per_metre_traversed[idx];
            my_twist[idx].twist.twist.linear.y = translation_delta.at<double>(1) * time_scale / pixels_per_metre_traversed[idx];
            my_twist[idx].twist.twist.angular.z = rotation_delta * rotation_velocity_scale[idx] * time_scale;
            my_twist[idx].header.stamp = current_time;
            
            pub[idx].publish(my_twist[idx]);
            
            ROS_INFO_STREAM("CAM " << idx);
            ROS_INFO_STREAM("lin x: " << my_twist[idx].twist.twist.linear.x);
            ROS_INFO_STREAM("lin y: " << my_twist[idx].twist.twist.linear.y);
            ROS_INFO_STREAM("ang z: " << my_twist[idx].twist.twist.angular.z);
            
            ++my_twist[idx].header.seq;
        }
    }
    prev_gray[idx] = frame_gray;
}

void optical_localization::image_callback0(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, 0);
}

void optical_localization::image_callback1(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, 1);
}

optical_localization::optical_localization(int argc, char *argv[]) {
    std::fill(pixels_per_metre_traversed, pixels_per_metre_traversed + NUM_CAMS, -1503.1124320333);
    std::fill(rotation_velocity_scale, rotation_velocity_scale + NUM_CAMS, 1.0);
    
    for(unsigned int i = 0;i < NUM_CAMS;++i) {
        translation_current[i] = (cv::Mat_<double>(2,1) << 0, 0);
        scale_x[i] = 1.0;
        scale_y[i] = 1.0;
        rotation[i] = 0.0;
        my_twist[i].twist.twist.linear.z = 0;
        my_twist[i].twist.twist.angular.x = 0;
        my_twist[i].twist.twist.angular.y = 0;
        my_twist[i].header.seq = 0;
        for(unsigned int j = 0;j < 36;++j) {
            my_twist[i].twist.covariance[j] = 0;
        }
        my_twist[i].header.frame_id = "base_link";
    }

    ros::init(argc, argv, "optical_localization");
    ros::NodeHandle n("~");
    
    sub[0] = n.subscribe("/optical_localization_cam0/image_raw", 1, &optical_localization::image_callback0, this);
    pub[0] = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/owr/optical_localization_twist0", 10, true);
    sub[1] = n.subscribe("/optical_localization_cam1/image_raw", 1, &optical_localization::image_callback1, this);
    pub[1] = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/owr/optical_localization_twist1", 10, true);
}

void optical_localization::run() {
    while(ros::ok()) {
        ros::spin();
    }
}

int main(int argc, char *argv[]) {
    optical_localization ol(argc, argv);
    ol.run();
    return 0;
}
