#ifndef OPTICAL_LOCALIZATION_H
#define OPTICAL_LOCALIZATION_H

#include <string>
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ecl/threads.hpp>

#define PIXELS_PER_METRE 1138.825332768

class optical_localization {
    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        
        cv::Mat prev_gray;
        
        ros::Time prev_time;
        
        // image displacement scale for each camera
        double pixels_per_metre;
        
        // axes mappings for each camera
        bool swap_axes;
        double axis_transforms[2];
        
        bool is_first;
        
        // most recent calculated twist for each camera
        geometry_msgs::Twist most_recent;
        
        // most recent averaged twist for each camera
        geometry_msgs::Twist most_recent_average;
        
        // the final twist to be published
        geometry_msgs::TwistWithCovarianceStamped my_twist;
        

        //ecl::Mutex mutex;
        //ros::AsyncSpinner asyncSpinner;
        
        std::string cam_names;
        
        void image_callback(const sensor_msgs::Image::ConstPtr& image);

        
        void process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int cam);
        
        geometry_msgs::Twist average(const geometry_msgs::Twist &first, const geometry_msgs::Twist &second);
        
        geometry_msgs::Twist getVectorSum();
        
        void publishTwist();
        void printTwist(const geometry_msgs::Twist &twist);
        
        // initialise pixels-to-metres scalar coefficients
        void assign_pixels_per_metre();
        
        // initialise pre-defined axis transformation factors
        void assign_axis_transforms();
        
        // initialise subscribers and publishers
        void assign_sub_pub();
        
        // helper to transform twists into the rover's frame of reference
        void align_axes(geometry_msgs::Twist &twist, const unsigned int cam);
        
    public:
        optical_localization();
        void run();
};

#endif // OPTICAL_LOCALIZATION_H
