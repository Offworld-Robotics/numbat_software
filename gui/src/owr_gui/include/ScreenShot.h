/* 
	Class header for ScreenShot
	Converting ROS images to OpenCV for modification
*/

#ifndef SCREENSHOT_H
#define SCREENSHOT_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
   
public:
ImageConverter() : it_(nh_) {
// Subscrive to input video feed and publish output video feed
	image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("/image_converter/output_video", 1);

     cv::namedWindow(OPENCV_WINDOW);
  27   }
  28 
  29   ~ImageConverter()
  30   {
  31     cv::destroyWindow(OPENCV_WINDOW);
  32   }
  33 
  34   void imageCb(const sensor_msgs::ImageConstPtr& msg)
  35   {
  36     cv_bridge::CvImagePtr cv_ptr;
  37     try
  38     {
  39       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  40     }
  41     catch (cv_bridge::Exception& e)
  42     {
  43       ROS_ERROR("cv_bridge exception: %s", e.what());
  44       return;
  45     }
  46 
  47     // Draw an example circle on the video stream
  48     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  49       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  50 
  51     // Update GUI Window
  52     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  53     cv::waitKey(3);
  54     
  55     // Output modified video stream
  56     image_pub_.publish(cv_ptr->toImageMsg());
  57   }
  58 };
  59 
  60 int main(int argc, char** argv)
  61 {
  62   ros::init(argc, argv, "image_converter");
  63   ImageConverter ic;
  64   ros::spin();
  65   return 0;
  66 }


#endif

