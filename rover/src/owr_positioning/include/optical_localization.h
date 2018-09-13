#ifndef OPTICAL_LOCALIZATION_H
#define OPTICAL_LOCALIZATION_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ecl/threads.hpp>

#define MAX_CAMS 4U
#define NUM_CAMS 4U
#define FRONT_CAM 0U
#define BACK_CAM 1U
#define LEFT_CAM 2U
#define RIGHT_CAM 3U

#define PIXELS_PER_METRE_FRONT 2568.5 //1138.825332768
#define PIXELS_PER_METRE_BACK 2568.5 //1138.825332768
#define PIXELS_PER_METRE_LEFT 2568.5 //1138.825332768
#define PIXELS_PER_METRE_RIGHT 2568.5 //1138.825332768

class optical_localization {
    private:
        ros::Publisher pub;
        ros::Subscriber sub[MAX_CAMS];
        
        cv::Mat prev_gray[MAX_CAMS];
        
        ros::Time prev_time[MAX_CAMS];
        
        // image displacement scale for each camera
        double pixels_per_metre[MAX_CAMS];
        
        // axes mappings for each camera
        bool swap_axes[MAX_CAMS];
        double axis_transforms[MAX_CAMS][2];
        
        bool is_first[MAX_CAMS];
        
        // most recent calculated twist for each camera
        geometry_msgs::Twist most_recent[MAX_CAMS];
        
        // most recent averaged twist for each camera
        geometry_msgs::Twist most_recent_average[MAX_CAMS];
        
        // the final twist to be published
        geometry_msgs::TwistWithCovarianceStamped my_twist;
        
        ecl::Mutex mutex;
        ros::AsyncSpinner asyncSpinner;
        
        std::string cam_names[MAX_CAMS];
        
        void image_callback_front(const sensor_msgs::CompressedImage::ConstPtr& image);
        void image_callback_back(const sensor_msgs::CompressedImage::ConstPtr& image);
        void image_callback_left(const sensor_msgs::CompressedImage::ConstPtr& image);
        void image_callback_right(const sensor_msgs::CompressedImage::ConstPtr& image);
        
        void process_image(const sensor_msgs::CompressedImage::ConstPtr& image, const unsigned int cam);
        
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
