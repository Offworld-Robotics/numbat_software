/*
 * Does computer vision on the clinometer to determine orientation
 * Original Author: Harry J.E Day
 * Editors: 
 * ROS_NODE: clinometer_node
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

 class ClinometerNode {
     
    public:
        ClinometerNode();
        
    //protected:
        
        void imageCallback(const sensor_msgs::Image::ConstPtr & msg);
        
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport imgTransport;
        
        image_transport::Subscriber camSub;
        ros::Publisher imuPub;
        
     
 };