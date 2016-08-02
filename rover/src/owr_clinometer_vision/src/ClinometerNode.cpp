/*
 * Does computer vision on the clinometer to determine orientation
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS_NODE: clinometer_node
 */

#include <owr_clinometer_vision/ClinometerNode.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#define RESOLUTION_PX 1

#define DEG_1 (CV_PI/180.0)
#define RESOLUTION_DEG DEG_1

#define MIN_THRESHOLD 50
#define MIN_LINE_LENGTH 50
#define MAX_LINE_GAP 10
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
        ROS_INFO("0");
        cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        ROS_INFO("a");
        cv::imshow("orig", img);
        cv::waitKey();
        cv::Mat cannyImg, colourImg, noBlue, noGreen;
        //cv::blur( img, img, cv::Size(3,3) );
	cv::MatIterator_<cv::Vec3b> it, end;
        for(it = img.begin<cv::Vec3b>(), end = img.end<cv::Vec3b>(); it != end; ++it) {
            if(((int)(*it)[0] + 20) > (*it)[2]   && ((int)(*it)[1]+20) > (*it)[2]) {
                for(int i = 0; i < img.channels(); ++i) {
                    (*it)[i] = 0;
                }
            }
        }
        cv::imshow("colourMask", img);
        cv::Mat channels[3];
        cv::split(img,channels);
        cv::imshow("red", channels[2]);
 
        cv::Canny(channels[2], cannyImg, 75, 200, 3);
        ROS_INFO("b");
        cv::cvtColor(cannyImg, colourImg, CV_GRAY2BGR);
	cv::imshow("canny", cannyImg);
        
        //convert the lines
        std::vector<cv::Vec4i> lines;
        HoughLinesP(cannyImg, lines, RESOLUTION_PX, RESOLUTION_DEG, MIN_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);
        
        #ifdef DEBUG
        //draw all the lines and display
        for( size_t i = 0; i < lines.size(); i++ ) {
            cv::Vec4i l = lines[i];
            cv::line( colourImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
        }
        
        cv::imshow("lines", colourImg);
        cv::waitKey();
        #endif
    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } catch(cv::Exception & e) {
        
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

