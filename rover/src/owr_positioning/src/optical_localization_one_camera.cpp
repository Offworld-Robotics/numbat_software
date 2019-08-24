#define _USE_MATH_DEFINES
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
#include <ecl/threads.hpp>
#include "optical_localization_one_camera.h"

void optical_localization::assign_pixels_per_metre() {
    // TODO change values according to cam
    pixels_per_metre = PIXELS_PER_METRE;
}

void optical_localization::assign_axis_transforms() {
    // swap axes values if needed, then reverse axes direction if needed
    swap_axes = true;
    axis_transforms[0] = 1;
    axis_transforms[1] = 1;
}

void optical_localization::align_axes(geometry_msgs::Twist &twist, const unsigned int cam) {
    // align camera's axes to rover's
    if(swap_axes) {
        std::swap(twist.linear.x, twist.linear.y);
    }
    twist.linear.x *= axis_transforms[0];
    twist.linear.y *= axis_transforms[1];
}

void optical_localization::assign_sub_pub() {
    ros::NodeHandle n("~");
    
    sub = n.subscribe("/optical_localization/image_raw", 1, &optical_localization::image_callback_front, this);
    
    pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/owr/optical_localization_twist", 10, true);
}

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
    
    result.linear.x += most_recent_average.linear.x;
    result.linear.y += most_recent_average.linear.y;
    result.angular.z += most_recent_average.angular.z;
    
    return result;
}

void optical_localization::printTwist(const geometry_msgs::Twist &twist) {
    ROS_INFO("lin x: %f, lin y: %f, ang z: %f", twist.linear.x, twist.linear.y, twist.angular.z);
}

void optical_localization::publishTwist() {
    if(is_first) {
        return;
    }
    my_twist.twist.twist = getVectorSum();
    my_twist.header.stamp = ros::Time::now();
    
    printTwist(my_twist.twist.twist);
    
    pub.publish(my_twist);
    ++my_twist.header.seq;
    
}

void optical_localization::process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int cam) {
    ROS_INFO_STREAM("Callback for cam " << cam << " - " <<  cam_names);
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

void optical_localization::image_callback_front(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, FRONT_CAM);
}

optical_localization::optical_localization(): asyncSpinner(0) {
    // TODO pixels_per_metre_traversed is different for each cam
    // TODO camera axes tranforms
    assign_pixels_per_metre();
    assign_axis_transforms();
    
    std::fill(is_first, is_first, true);
    
    std::fill(my_twist.twist.covariance.begin(), my_twist.twist.covariance.end(), 0.0);
    my_twist.header.seq = 0;
    my_twist.header.frame_id = "base_link";
    
    cam_names = "front";
    
    assign_sub_pub();
}

void optical_localization::run() {
    asyncSpinner.start();
    ros::waitForShutdown();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "optical_localization");
    optical_localization ol;
    ol.run();
    return 0;
}
