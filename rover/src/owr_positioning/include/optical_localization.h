#ifndef OPTICAL_LOCALIZATION_H
#define OPTICAL_LOCALIZATION_H

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ecl/threads.hpp>

#define NUM_CAMS 4U
#define FRONT_CAM 0U
#define BACK_CAM 1U
#define LEFT_CAM 2U
#define RIGHT_CAM 3U

#define PIXELS_PER_METRE_FRONT -1503.1124320333
#define PIXELS_PER_METRE_BACK -1503.1124320333
#define PIXELS_PER_METRE_LEFT -1503.1124320333
#define PIXELS_PER_METRE_RIGHT -1503.1124320333

class optical_localization {
    private:
        ros::Publisher pub;
        ros::Subscriber sub[NUM_CAMS];
        
        cv::Mat prev_gray[NUM_CAMS];
        
        ros::Time prev_time[NUM_CAMS];
        
        // image displacement scale for each camera
        double pixels_per_metre[NUM_CAMS];
        
        // axes mappings for each camera
        bool swap_axes[NUM_CAMS];
        double axis_transforms[NUM_CAMS][2];
        
        bool is_first[NUM_CAMS];
        
        // most recent calculated twist for each camera
        geometry_msgs::Twist most_recent[NUM_CAMS];
        
        // most recent averaged twist for each camera
        geometry_msgs::Twist most_recent_average[NUM_CAMS];
        
        // the final twist to be published
        geometry_msgs::TwistWithCovarianceStamped my_twist;
        
        ecl::Mutex mutex;
        
        void image_callback_front(const sensor_msgs::Image::ConstPtr& image);
        void image_callback_back(const sensor_msgs::Image::ConstPtr& image);
        void image_callback_left(const sensor_msgs::Image::ConstPtr& image);
        void image_callback_right(const sensor_msgs::Image::ConstPtr& image);
        
        void process_image(const sensor_msgs::Image::ConstPtr& image, const unsigned int cam);
        
        geometry_msgs::Twist average(const geometry_msgs::Twist &first, const geometry_msgs::Twist &second);
        
        geometry_msgs::Twist getVectorSum();
        
        void publishTwist();
        
        void assign_pixels_per_metre();
        void assign_axis_transforms();
        
        void assign_sub_pub();
        
    public:
        optical_localization(int argc, char *argv[]);
        void run();
};

#endif // OPTICAL_LOCALIZATION_H
