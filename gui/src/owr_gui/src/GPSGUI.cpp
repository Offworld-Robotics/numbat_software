/*
 * GPSLogger Node
 * Logs GPS input to KML
 * By Harry J.E Day for Bluesat OWR
 * Date: 31/05/2014
 */
#include "GpsGUI.h"
#include <fstream>
#include "comms.h"


GPSLogger::GPSLogger() {
    ROS_INFO("initlising GPSLogger");
    //a nodehandler is used to communiate with the rest of ros
    ros::NodeHandle n("~");

    //pass the function that is called when a message is recived
    sub = n.subscribe("/gps/fix", 1000, &GPSLogger::reciveMsg, this);
   
    
}

void GPSLogger::spin() {
  
  ros::spin();

}



void GPSLogger::reciveMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    assert(msg);
    
    ROS_INFO("recived a message");
    ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
        
        
    
}


