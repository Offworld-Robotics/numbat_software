/*
 * Date Started: 24/8/18
 * Original Author: Alan Nguyen
 * Editors: Edward Dai
 * ROS Node Name: image_cropping
 * ROS Package: owr_positioning
 * Purpose: Crops subscribed images to a defined size and re-publishes them
 * This code is released under the MIT [GPL for embeded] License. Copyright BLUEsat UNSW, 2017
 */

#ifndef IMAGE_CROPPING_H
#define IMAGE_CROPPING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ImageCropping {
    public:
        ImageCropping();
        void run();
        void rawImageCallback(const sensor_msgs::Image::ConstPtr & msg);

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
