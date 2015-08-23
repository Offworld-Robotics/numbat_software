/*
 * Main class for pbuff relays
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "PositionController.h" 
#include <iostream>
#include <list>
#include <cmath>
#include <stdio.h>

#define TOPIC "/owr/position"
//minum number of lat/long inputs to calculate the heading
#define MIN_H_CALC_BUFFER_SIZE 2 
 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_position_node");
    
    PositionController p(TOPIC);
    p.spin();
    
    return EXIT_SUCCESS;   
}

PositionController::PositionController(const std::string topic) {
    altitude = 0;
    latitude = 0;
    longitude = 0;
    pitch = 0;
    roll = 0;
    heading = 0;
    publisher =  node.advertise<owr_messages::position>(topic,1000,true);
    gpsSubscriber = node.subscribe("/gps/fix", 2, &PositionController::receiveGPSMsg, this); // GPS related data
}

void PositionController::receiveGPSMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
    altitude  = msg->altitude;
    latitude  = msg->latitude;
    longitude = msg->longitude;
    std::list<double>::iterator latItr = latitudes.begin();
    double x1 = *(latItr);
    std::list<double>::iterator lonItr = longitudes.begin();
    double y1 = *(lonItr);
    if (x1 == latitude && y1 == longitude) {
        latitudes.push_front(latitude);
        longitudes.push_front(longitude);
    }
    updateHeading();
    sendMsg();
}

/**
 * This calulates the heading based on the current, and previouse longitude and latitude
 */
void PositionController::updateHeading() {
    //check we have enought data
    if(latitudes.size() >= MIN_H_CALC_BUFFER_SIZE) {

        std::list<double>::iterator latItr = latitudes.begin();
        double x1 = *(latItr);
        double x2 = *(++latItr);
        std::list<double>::iterator lonItr = longitudes.begin();
        double y1 = *(lonItr);
        double y2 = *(++lonItr);
	ROS_INFO("%f %f and %f %f", x1, x2, y1, y2);
        //this formula from http://www.moveable-type.co.uk/scripts/latlong.html
        //should calculate the bearing between two points.
        //TODO: check that this is correct.
        double y = sin(x2-x1) * cos(y2);
        //I'm pretty sure this can be simplified..
        double x = cos(y1)*sin(y2) - sin(y1)*cos(y2)*cos(y2-y1);
        heading = atan2(y,x) * 180/M_PI;
        //old formula using arc tan
        //heading = atan((y1 - y2) / (x1 - x2)) * M_PI / 180;
        std::cout << "heading " << heading << "deg\n";
    }
}

void PositionController::sendMsg() {
    owr_messages::position msg;
    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude = altitude;
    msg.pitch = pitch;
    msg.roll = roll;
    msg.heading = heading;
    
    publisher.publish(msg);
}


//main loop
void PositionController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
