#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "ImageCropping.hpp"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "image_cropping");
    ImageCropping imgCrop;
    imgCrop.run();
}

ImageCropping::ImageCropping() : nh(), it(nh) {
    ros::NodeHandle n("~"); // private handle for parameters
    if (!n.getParam("x", x)) {
        ROS_INFO("Failed to get param 'x'");
    }
    n.getParam("y", y);
    n.getParam("crop_width", crop_width);
    n.getParam("crop_height", crop_height);
    ROS_INFO("Parameters for crop: x=%d y=%d width=%d height=%d", x,y,crop_width,crop_height);
    img_raw_sub = it.subscribe("/camera/image_raw", 1, &ImageCropping::rawimageCallback, this);
    img_cropped_pub = it.advertise("/camera/image_cropped", 1);
}

void ImageCropping::run() {
    while(ros::ok()) {
        ros::spin();
    }
}

void ImageCropping::rawimageCallback(const sensor_msgs::Image::ConstPtr & msg) {
    try {
        // copy image with source encoding
        cv::Mat img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
        // crop image
        cv::Rect rect(x, y, crop_width, crop_height);
        img_crop = img(rect).clone(); // create new copy of image
        //cv::imshow("cropped", img_crop);
        //cv::waitKey(0);
        cv::Size size = img_crop.size();
        ROS_INFO("Published cropped size = %d x %d", size.height, size.width);
        // create ros image
        sensor_msgs::ImagePtr msg_crop = cv_bridge::CvImage(std_msgs::Header(),
                                        msg->encoding, img_crop).toImageMsg();
        img_cropped_pub.publish(msg_crop);        
    } catch (cv_bridge::Exception e) {
        ROS_ERROR("Could not convert image encoding");
    }
}
