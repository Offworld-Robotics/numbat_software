
#include "owr_gps_logging/gpsLogger.h"

int main(int argc, char** argv) { 
    
    //initialise the ros package
    ros::init(argc, argv, "gpsLogger");
    GPSLogger gpsLogger;
    
    //loop with callbacks
    ros::spin();
    return EXIT_SUCCESS;
}

GPSLogger GPSLogger::GPSLoger() {
    //a nodehandler is used to communiate with the rest of ros
    ros::NodeHandle n;

    //pass the function that is called when a message is recived
    ros::Subscriber sub = n.subscribe("/gps/fix", 1000, reciveMsg);

    
}



void GPSLogger::reciveMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    
    assert(msg);
    
    ROS_INFO("recived a message");
    const int arraySize = 4 ;
    ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg-altitude);
    
    //open the file
    std::ofstream kmlFile; 
    kmlFile.open(LOG_FILE);
    kmlFile << KML_HEADER;
    //TODO: add in co-ords
    kmlFile<<KML_FOOTER;
    kmlFile.close();
}


