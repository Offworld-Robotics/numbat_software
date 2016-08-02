/*
 * Does computer vision on the clinometer to determine orientation
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS_NODE: clinometer_node
 */

#include <owr_clinometer_vision/ClinometerNode.hpp>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

#define RESOLUTION_PX 3

#define DEG_1 (CV_PI/180.0)
#define RESOLUTION_DEG DEG_1

#define MIN_THRESHOLD 50
#define MIN_LINE_LENGTH 55
#define MAX_LINE_GAP 12
#define BG_INTENSITY_THRESHOLD 150

#define DEBUG

int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "owr_clinometer_vision");
    ClinometerNode cn;
    
#ifdef DEBUG_TEST
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/hjed/Bluesat/owr_software/rover/src/owr_clinometer_vision/617G1jejJIL._SL1500_.jpg",CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    cn.imageCallback(boost::const_pointer_cast<sensor_msgs::Image>(boost::shared_ptr<sensor_msgs::Image>( &ros_image)));
#endif
    
    ros::spin();
}

ClinometerNode::ClinometerNode() : nh(), imgTransport(nh) {
    camSub = imgTransport.subscribe("/clinometer_cam/image_raw", 1, &ClinometerNode::imageCallback, this);
    imuPub = nh.advertise<sensor_msgs::Imu>("/owr/sensors/clinometer", 2, true);
}

void ClinometerNode::imageCallback(const sensor_msgs::Image_< std::allocator< void > >::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        //crop the image
        cv::Rect rect(50,100,490, 300);
        img = img(rect);
        //source: http://stackoverflow.com/questions/35866411/opencv-how-to-detect-lines-of-a-specific-colour

        cv::Mat hsvImg;
        cv::cvtColor(img, hsvImg, CV_BGR2HSV);        
        cv::imshow("hsv", hsvImg);
        
        //split the hsv channels
        cv::Mat channels[3];
        cv::split(hsvImg, channels);

       //red is painfull because it is in the end of the hsv colour circle
       const int hueValue = 0;
       const int hueRange = 15;
       
       const int MIN_SATURATION = 70;
       const int MIN_VALUE  = 100;
       
       //check if the colour is in the lower hue range
       cv::Mat hueMask;
       cv::inRange(channels[0], hueValue - hueRange, hueValue + hueRange, hueMask);
       
       //we need to check the other side to because red
       if(hueValue - hueRange < 0 || hueValue + hueRange > 180) {
           cv::Mat hueMaskUpper;
           int upperHueValue = hueValue + 180;
           cv::inRange(channels[0], upperHueValue - hueRange, upperHueValue + hueRange, hueMaskUpper);

           //bitwise or the masks
           hueMask = hueMask | hueMaskUpper;
       }

      //filter out saturation and value limits
      cv::Mat satMask = channels[1] > MIN_SATURATION;
      cv::Mat valueMask = channels[2] > MIN_VALUE;

      //apply a mask to filter
      hueMask = (hueMask & satMask) & valueMask;
      cv::imshow("red colour", hueMask);
      

      //apply a Canny filter so we get edges of lines
      cv::Mat cannyImg;
      cv::Canny(hueMask, cannyImg, 50, 200, 3);
#ifdef DEBUG
      cv::imshow("canny", cannyImg);
#endif
      std::vector<cv::Vec4i> lines;
      HoughLinesP(cannyImg, lines, RESOLUTION_PX, RESOLUTION_DEG, MIN_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);
        
        #ifdef DEBUG
        //draw all the lines and display
        for( size_t i = 0; i < lines.size(); i++ ) {
            cv::Vec4i l = lines[i];
            cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
        }
        
        cv::imshow("lines", img);
        #endif
      //calculate the gradients of the lines
      std::vector<double> gradients;
      for(size_t i =0; i < lines.size(); i++) {
          const cv::Vec4i l = lines[i];
          //gradient is (y1-y2)/(x1-x2)
          gradients.push_back(((float)(l[0]-l[2]))/(l[1]-l[3]));
#ifdef DEBUG
          ROS_INFO("%f gradient", gradients.back());
#endif
      }

      sensor_msgs::Imu imuMsg;
      imuMsg.header = msg->header;
      //TODO: set covariance on angular vel and lin acc to -1
      int roll = atan(gradients[0]);
      int pitch = atan(gradients[1]);
      tf::Quaternion quat(roll, pitch,0);
      imuMsg.orientation.x = quat.x();
      imuMsg.orientation.y = quat.y();
      imuMsg.orientation.z = quat.z();
      imuMsg.orientation.w = quat.w();
      imuPub.publish(imuMsg);
      cv::waitKey();

    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } catch(cv::Exception & e) {
        
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

