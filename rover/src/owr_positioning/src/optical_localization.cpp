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
#include "optical_localization.h"

void optical_localization::assign_pixels_per_metre() {
    // TODO change values according to cam
    pixels_per_metre[FRONT_CAM] = PIXELS_PER_METRE_FRONT;
    pixels_per_metre[BACK_CAM] = PIXELS_PER_METRE_BACK;
    pixels_per_metre[LEFT_CAM] = PIXELS_PER_METRE_LEFT;
    pixels_per_metre[RIGHT_CAM] = PIXELS_PER_METRE_RIGHT;
}

void optical_localization::assign_axis_transforms() {
    // TODO make sure this is right
    
    // swap axes values if applicable, then multiply each axis by transform
    swap_axes[FRONT_CAM] = false;
    axis_transforms[FRONT_CAM][0] = 1;
    axis_transforms[FRONT_CAM][1] = 1;
    
    swap_axes[BACK_CAM] = false;
    axis_transforms[BACK_CAM][0] = -1;
    axis_transforms[BACK_CAM][1] = -1;
    
    swap_axes[LEFT_CAM] = true;
    axis_transforms[LEFT_CAM][0] = -1;
    axis_transforms[LEFT_CAM][1] = 1;
    
    swap_axes[RIGHT_CAM] = true;
    axis_transforms[RIGHT_CAM][0] = 1;
    axis_transforms[RIGHT_CAM][1] = -1;
}

void optical_localization::assign_sub_pub() {
    ros::NodeHandle n("~");
    char topic[50];
    
    sprintf(topic, "/optical_localization_cam%d/image_raw", FRONT_CAM);
    sub[FRONT_CAM] = n.subscribe(topic, 1, &optical_localization::image_callback_front, this);
    
    sprintf(topic, "/optical_localization_cam%d/image_raw", BACK_CAM);
    sub[BACK_CAM] = n.subscribe(topic, 1, &optical_localization::image_callback_back, this);
    
    sprintf(topic, "/optical_localization_cam%d/image_raw", LEFT_CAM);
    sub[LEFT_CAM] = n.subscribe(topic, 1, &optical_localization::image_callback_left, this);
    
    sprintf(topic, "/optical_localization_cam%d/image_raw", RIGHT_CAM);
    sub[RIGHT_CAM] = n.subscribe(topic, 1, &optical_localization::image_callback_right, this);
    
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
    
    for(unsigned int i = 0;i < NUM_CAMS;++i) {
        result.linear.x += most_recent_average[i].linear.x;
        result.linear.y += most_recent_average[i].linear.y;
        result.angular.z += most_recent_average[i].angular.z;
    }
    result.linear.x /= NUM_CAMS;
    result.linear.y /= NUM_CAMS;
    result.angular.z /= NUM_CAMS;
    
    return result;
}

void optical_localization::printTwist(const geometry_msgs::Twist &twist) {
    ROS_INFO_STREAM("lin x: " << twist.linear.x);
    ROS_INFO_STREAM("lin y: " << twist.linear.y);
    ROS_INFO_STREAM("ang z: " << twist.angular.z);
}

void optical_localization::publishTwist() {
    mutex.lock();
    for(unsigned int i = 0;i < NUM_CAMS;++i) {
        if(is_first[i]) {
            mutex.unlock();
            return;
        }
    }
    my_twist.twist.twist = getVectorSum();
    my_twist.header.stamp = ros::Time::now();
    
    ROS_INFO_STREAM("Publishing:");
    printTwist(my_twist.twist.twist);
    
    pub.publish(my_twist);
    ++my_twist.header.seq;
    
    mutex.unlock();
}

void optical_localization::process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int cam) {
    ROS_INFO_STREAM("Callback for " << cam);
    cv::Mat frame = cv_bridge::toCvCopy(image)->image;
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    
    if(!prev_gray[cam].empty()) {
        cv::Mat affineTransform = estimateRigidTransform(prev_gray[cam], frame_gray, false);
        if(!affineTransform.empty()) {
            cv::Mat translation_delta = affineTransform.col(2);
            double scale_x_delta = norm(affineTransform.col(0));
            double scale_y_delta = norm(affineTransform.col(1));
            double rotation_delta = atan2(affineTransform.at<double>(1,1), affineTransform.at<double>(0,1)) - M_PI/2.0;
            
            double time_scale = 0;
            ros::Time current_time = ros::Time::now();
            if(!prev_time[cam].isZero()) {
                time_scale = (1.0 / ((current_time - prev_time[cam]).nsec / 1e9));
            }
            prev_time[cam] = current_time;
            
            geometry_msgs::Twist this_twist;
            this_twist.linear.x = translation_delta.at<double>(0) * time_scale / pixels_per_metre[cam];
            this_twist.linear.y = translation_delta.at<double>(1) * time_scale / pixels_per_metre[cam];
            this_twist.angular.z = rotation_delta * time_scale;
            
            // align axes to rover
            if(swap_axes[cam]) {
                std::swap(this_twist.linear.x, this_twist.linear.y);
            }
            this_twist.linear.x *= axis_transforms[cam][0];
            this_twist.linear.y *= axis_transforms[cam][1];
            
            if(is_first[cam]) {
                is_first[cam] = false;
                most_recent_average[cam] = this_twist;
            } else {
                most_recent_average[cam] = average(most_recent[cam], this_twist);
            }
            //printTwist(this_twist);
            most_recent[cam] = this_twist;
            
            publishTwist();
        }
    }
    prev_gray[cam] = frame_gray;
}

void optical_localization::image_callback_front(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, FRONT_CAM);
}

void optical_localization::image_callback_back(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, BACK_CAM);
}

void optical_localization::image_callback_left(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, LEFT_CAM);
}

void optical_localization::image_callback_right(const sensor_msgs::Image::ConstPtr& image) {
    process_image(image, RIGHT_CAM);
}

optical_localization::optical_localization(): asyncSpinner(0) {
    // TODO pixels_per_metre_traversed is different for each cam
    // TODO camera axes tranforms
    assign_pixels_per_metre();
    assign_axis_transforms();
    
    std::fill(is_first, is_first + NUM_CAMS, true);
    
    std::fill(my_twist.twist.covariance.begin(), my_twist.twist.covariance.end(), 0.0);
    my_twist.header.seq = 0;
    my_twist.header.frame_id = "base_link";
    
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
