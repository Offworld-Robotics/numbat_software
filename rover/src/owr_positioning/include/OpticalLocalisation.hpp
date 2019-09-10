#pragma once
#include <string>
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

class OpticalLocalisation {
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Time prev_time;
        cv::Mat prev_gray;
        int seq;

        void process_image(const sensor_msgs::Image::ConstPtr& image);

    public:
        OpticalLocalisation();
        void run();
};

