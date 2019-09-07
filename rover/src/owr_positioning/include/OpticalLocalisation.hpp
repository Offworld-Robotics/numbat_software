
#pragma once 

#include <string>
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// ????
#define PIXELS_PER_METRE 1138.825332768

class OpticalLocalisation {
    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        cv::Mat prev_gray;
        ros::Time prev_time;

        void process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int cam);
        void printTwist(const geometry_msgs::Twist &twist);

    public:
        OpticalLocalisation();
        void run();
};

