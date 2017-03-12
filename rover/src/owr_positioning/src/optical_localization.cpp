#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ecl/threads.hpp>
#include "optical_localization.h"

geometry_msgs::Twist optical_localization::average(const geometry_msgs::Twist &first, const geometry_msgs::Twist &second) {
    geometry_msgs::Twist result;
    result.linear.x = (first.linear.x + second.linear.x) / 2.0;
    result.linear.y = (first.linear.y + second.linear.y) / 2.0;
    result.angular.z = (first.angular.z + second.angular.z) / 2.0;
    return result;
}

geometry_msgs::Twist optical_localization::getVectorSum() {
    geometry_msgs::Twist result;
    
    // TODO do actual vector sum
    result.linear.x = 0;
    result.linear.y = 0;
    result.angular.z = 0;
    
    for(unsigned int i = 0;i < NUM_CAMS;++i) {
        result.linear.x += most_recent_averages[i].linear.x;
        result.linear.y += most_recent_averages[i].linear.y;
        result.angular.z += most_recent_averages[i].angular.z;
    }
    result.linear.x /= NUM_CAMS;
    result.linear.y /= NUM_CAMS;
    result.angular.z /= NUM_CAMS;
    
    return result;
}

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
            
            double time_scale = 0;
            ros::Time current_time = ros::Time::now();
            if(!prev_time[idx].isZero()) {
                time_scale = (1.0 / ((current_time - prev_time[idx]).nsec / 1e9));
            }
            prev_time[idx] = current_time;
            
            geometry_msgs::Twist this_twist;
            this_twist.linear.x = translation_delta.at<double>(0) * time_scale / pixels_per_metre_traversed[idx];
            this_twist.linear.y = translation_delta.at<double>(1) * time_scale / pixels_per_metre_traversed[idx];
            this_twist.angular.z = rotation_delta * rotation_velocity_scale[idx] * time_scale;
            
            if(is_first[idx]) {
                is_first[idx] = false;
                most_recent_averages[idx] = this_twist;
            } else {
                most_recent_averages[idx] = average(most_recent[idx], this_twist);
            }
            most_recent[idx] = this_twist;
            
            // TODO enable multithreading
            mutex.lock();
            my_twist.twist.twist = getVectorSum();
            my_twist.header.stamp = ros::Time::now();
            
            pub.publish(my_twist);
            ++my_twist.header.seq;
            
            ROS_INFO_STREAM("CAM " << idx);
            ROS_INFO_STREAM("lin x: " << my_twist.twist.twist.linear.x);
            ROS_INFO_STREAM("lin y: " << my_twist.twist.twist.linear.y);
            ROS_INFO_STREAM("ang z: " << my_twist.twist.twist.angular.z);
            
            mutex.unlock();
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

void optical_localization::image_callback2(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, 2);
}

void optical_localization::image_callback3(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, 3);
}

optical_localization::optical_localization(int argc, char *argv[]) {
    // TODO pixels_per_metre_traversed is different for each cam
    // TODO camera axes tranforms
    std::fill(pixels_per_metre_traversed, pixels_per_metre_traversed + NUM_CAMS, -1503.1124320333);
    std::fill(rotation_velocity_scale, rotation_velocity_scale + NUM_CAMS, 1.0);
    std::fill(is_first, is_first + NUM_CAMS, true);

    ros::init(argc, argv, "optical_localization");
    ros::NodeHandle n("~");
    
    my_twist.header.seq = 0;
    my_twist.header.frame_id = "base_link";
    
    for(unsigned int i = 0;i < NUM_CAMS;++i) {
        my_twist.twist.covariance[i] = 0;
    }
    
    sub[0] = n.subscribe("/optical_localization_cam0/image_raw", 1, &optical_localization::image_callback0, this);
    sub[1] = n.subscribe("/optical_localization_cam1/image_raw", 1, &optical_localization::image_callback1, this);
    sub[2] = n.subscribe("/optical_localization_cam2/image_raw", 1, &optical_localization::image_callback2, this);
    sub[3] = n.subscribe("/optical_localization_cam3/image_raw", 1, &optical_localization::image_callback3, this);
    
    pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/owr/optical_localization_twist", 10, true);
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
