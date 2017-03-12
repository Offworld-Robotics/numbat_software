#ifndef OPTICAL_LOCALIZATION_H
#define OPTICAL_LOCALIZATION_H

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ecl/threads.hpp>

#define NUM_CAMS 4U

class optical_localization {
    private:
        ros::Publisher pub;
        
        ros::Subscriber sub[NUM_CAMS];
        
        cv::Mat prev_gray[NUM_CAMS];
        
        ros::Time prev_time[NUM_CAMS];
        
        double pixels_per_metre_traversed[NUM_CAMS];
        double rotation_velocity_scale[NUM_CAMS];
        
        bool is_first[NUM_CAMS];
        
        // most recent calculated twist for each camera
        geometry_msgs::Twist most_recent[NUM_CAMS];
        
        // most recent averaged twist for each camera
        geometry_msgs::Twist most_recent_averages[NUM_CAMS];
        
        // the final twist to be published
        geometry_msgs::TwistWithCovarianceStamped my_twist;
        
        ecl::Mutex mutex;
        
        void image_callback0(const sensor_msgs::Image::ConstPtr& image);
        void image_callback1(const sensor_msgs::Image::ConstPtr& image);
        void image_callback2(const sensor_msgs::Image::ConstPtr& image);
        void image_callback3(const sensor_msgs::Image::ConstPtr& image);
        
        void process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int idx);
        
        geometry_msgs::Twist average(const geometry_msgs::Twist &first, const geometry_msgs::Twist &second);
        
        geometry_msgs::Twist getVectorSum();
        
    public:
        optical_localization(int argc, char *argv[]);
        void run();
    
};

#endif // OPTICAL_LOCALIZATION_H
