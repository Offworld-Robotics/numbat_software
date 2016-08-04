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

#define MIN_THRESHOLD 40
#define MIN_LINE_LENGTH 20
#define MAX_LINE_GAP 15
#define BG_INTENSITY_THRESHOLD 150

#define COVARIANCE_SIZE 9

#define DEBUG

static inline void zeroCovariances(sensor_msgs::Imu & imu);
static double doLineDetection(cv::Mat img);

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
        cv::Rect rect(200,100,340, 300);
        img = img(rect);
        //source: http://stackoverflow.com/questions/35866411/opencv-how-to-detect-lines-of-a-specific-colour

        cv::Mat hsvImg;
        cv::cvtColor(img, hsvImg, CV_BGR2HSV);
#ifdef DEBUG
        cv::imshow("hsv", hsvImg);
#endif

        //split the hsv channels
        cv::Mat channels[3];
        cv::split(hsvImg, channels);

        //red is painfull because it is in the end of the hsv colour circle
        const int HUE_VALUE = 0;
        const int HUE_RANGE = 12;

        const int MIN_SATURATION = 60;
        const int MIN_VALUE  = 100;

        //check if the colour is in the lower hue range
        cv::Mat hueMask;
        cv::inRange(channels[0], HUE_VALUE - HUE_RANGE, HUE_VALUE + HUE_RANGE, hueMask);

        //we need to check the other side to because red
        //if statement for readability only, will always run
        if(HUE_VALUE - HUE_RANGE < 0 || HUE_VALUE + HUE_RANGE > 180) {
            cv::Mat hueMaskUpper;
            const int UPPER_HUE_VALUE = HUE_VALUE + 180;
            cv::inRange(channels[0], UPPER_HUE_VALUE - HUE_RANGE, UPPER_HUE_VALUE + HUE_RANGE, hueMaskUpper);

            //bitwise or the masks
            hueMask = hueMask | hueMaskUpper;
        }

        //filter out saturation and value limits
        cv::Mat satMask = channels[1] > MIN_SATURATION;
        cv::Mat valueMask = channels[2] > MIN_VALUE;

        //apply a mask to filter
        hueMask = (hueMask & satMask) & valueMask;
#ifdef DEBUG
        cv::imshow("red colour", hueMask);
#endif
        //denoising
        cv::medianBlur(hueMask,hueMask,5);
#ifdef DEBUG
        cv::imshow("denoise", hueMask);
#endif
        //gradient
        cv::Mat rollImg, pitchImg;
        rollImg = hueMask.clone();
        pitchImg = hueMask.clone();
        ROS_INFO("x %d, y %d", rollImg.size().width, rollImg.size().height);
        const cv::Rect rollRect(0, 80 , 150,80 );
        double roll  = doLineDetection(rollImg(rollRect));
        const cv::Rect pitchRect(170, 80 , 150,80 );
        double pitch = doLineDetection(pitchImg(pitchRect));

        sensor_msgs::Imu imuMsg;
        imuMsg.header = msg->header;
        zeroCovariances(imuMsg);
        /*roll = angles[0];
        } else {
           ROS_ERROR("No gradient found");
        }
        int pitch = angles[1];*/
        tf::Quaternion quat(roll, pitch,0);
        imuMsg.orientation.x = quat.x();
        imuMsg.orientation.y = quat.y();
        imuMsg.orientation.z = quat.z();
        imuMsg.orientation.w = quat.w();
        imuPub.publish(imuMsg);
#ifdef DEBUG
        cv::waitKey();
#endif

    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } catch(cv::Exception & e) {

        ROS_ERROR("cv exception: %s", e.what());
        return;
    }
}


static inline void zeroCovariances(sensor_msgs::Imu & imu) {
    for(int i = 0; i < COVARIANCE_SIZE; ++i) {
        imu.angular_velocity_covariance[i] = -1;
        imu.linear_acceleration_covariance[i] = -1;
    }

}



static double doLineDetection(cv::Mat img) {
    const float MAX_ANGLE = 0.872665; //50deg in radians
    const float GRADIENT_MATCH_ERROR = 0.1; //1 radian error margin
    const float CIRC_RADIUS = 10;

    //apply a Canny filter so we get edges of lines
    cv::Mat cannyImg;
    cv::Canny(img, cannyImg, 50, 200, 3);
#ifdef DEBUG
    cv::imshow("canny", cannyImg);
#endif
    std::vector<cv::Vec4i> lines;
    HoughLinesP(cannyImg, lines, RESOLUTION_PX, RESOLUTION_DEG, MIN_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);

#ifdef DEBUG
    cv::Mat debugImg;
    cv::cvtColor(img, debugImg, CV_GRAY2RGB);
    //draw all the lines and display
    for( size_t i = 0; i < lines.size(); i++ ) {
        cv::Vec4i l = lines[i];
        cv::line(debugImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    }
#endif

    //calculate the angles of the lines
    std::vector<double> angles;
    std::vector<cv::Vec4i> goodLines;
    for(size_t i =0; i < lines.size(); i++) {
        const cv::Vec4i l = lines[i];
        
        //gradient is (y1-y2)/(x1-x2)
        double angle = atan(((float)(l[1]-l[3])/(l[0]-l[2]))); 
#ifdef DEBUG
        ROS_INFO("%f gradient", angle);
#endif
        if (fabs(angle) < MAX_ANGLE) {
            angles.push_back(angle);
            goodLines.push_back(l);
#ifdef DEBUG
            cv::line(debugImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 3, CV_AA);
#endif
        }
    }
    
    
#ifdef DEBUG
    cv::imshow("lines", debugImg);
    ROS_INFO("Next img");
    cv::waitKey();
#endif
    if(angles.size() > 0) {
        std::vector<double> chosenAngles;
        for(int i = 0; i < angles.size(); ++i) {
            for(int j = 0; j < angles.size(); ++j) {
                if(fabs(angles[i] - angles[j]) < GRADIENT_MATCH_ERROR) {
                    //check if the line passes through the center (ish)
                    //use the shortest distance from point to line formula
                    //http://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
                    const int centerY = cannyImg.size().height/2;
                    const int centerX = cannyImg.size().height/2;
                    const int numerator = fabs(
                        (goodLines[j][2] - goodLines[i][0])*centerX +
                        (goodLines[j][3] - goodLines[i][1])*centerY +
                        (goodLines[i][0] - goodLines[j][2])*goodLines[i][1] +
                        (goodLines[i][1] - goodLines[j][3])*goodLines[j][0] 
                    );
                    const int denominator = sqrt(
                        pow(goodLines[j][2] - goodLines[i][0], 2) +
                        pow(goodLines[i][1] - goodLines[j][3], 2)
                    );
                    if((numerator/denominator) <= CIRC_RADIUS) {
                        chosenAngles.push_back(angles[i]);
#ifdef DEBUG
                        cv::line(debugImg, cv::Point(goodLines[i][0], goodLines[i][1]), cv::Point(goodLines[j][2], goodLines[j][3]), cv::Scalar(255,0,0), 3, CV_AA);
#endif
                    }
                }
            }
        }
        if(chosenAngles.size() > 0) {
            return chosenAngles[0];
        } else {
            ROS_ERROR("No Good lines found");
            return std::numeric_limits<double>::infinity() * -1;
        }
    } else {
        ROS_ERROR("No lines found");
        return std::numeric_limits<double>::infinity();
    }

}
