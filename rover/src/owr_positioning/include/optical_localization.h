#ifndef OPTICAL_LOCALIZATION_H
#define OPTICAL_LOCALIZATION_H

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#define NUM_CAMS 2U

class optical_localization {
    private:
        ros::Subscriber sub[NUM_CAMS];
        ros::Publisher pub[NUM_CAMS];
        
        cv::Mat prev_gray[NUM_CAMS];
        cv::Mat translation_current[NUM_CAMS];
        double scale_x[NUM_CAMS];
        double scale_y[NUM_CAMS];
        double rotation[NUM_CAMS];
        
        ros::Time prev_time[NUM_CAMS];
        
        double pixels_per_metre_traversed[NUM_CAMS];
        double rotation_velocity_scale[NUM_CAMS];
        
        geometry_msgs::TwistWithCovarianceStamped my_twist[NUM_CAMS];
        
        void image_callback(const sensor_msgs::Image::ConstPtr& image);
        
        void process_image(unsigned int cam_index);
        
    public:
        optical_localization(int argc, char *argv[]);
        void run();
    
};

#endif // OPTICAL_LOCALIZATION_H
