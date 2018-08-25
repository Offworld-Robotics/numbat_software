/*
 * Crops subscribed images to a defined size and re-publishes them
 * Original Author:
 * Editors:
 * ROS_NODE: image_cropping
 */

#ifndef IMAGE_CROPPING_H
#define IMAGE_CROPPING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ImageCropping {
    public:
        ImageCropping();
        void run();
        void rawimageCallback(const sensor_msgs::Image::ConstPtr & msg);

    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber img_raw_sub;
        image_transport::Publisher img_cropped_pub;
        cv::Mat img_crop;
        int x, y;
        int crop_width, crop_height;
};

#endif
