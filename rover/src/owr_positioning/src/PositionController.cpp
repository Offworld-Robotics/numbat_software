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
    gpsSubscriber = node.subscribe("/gps/fix", 1000, &PositionController::receiveGPSMsg, this); // GPS related data
}

void PositionController::receiveGPSMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
    altitude  = msg->altitude;
    latitude  = msg->latitude;
    longitude = msg->longitude;
    latitudes.push_front(latitude);
    longitudes.push_front(longitude);
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
        
        heading = tanh((y1 - y2) / (x1 - x2)) * M_PI;
        std::cout << "heading " << heading << "deg\n";
    }
}

void PositionController::sendMsg() {
    owr_messages::position msg;
    msg.latitude = latitude;
    msg.longitude = latitude;
    msg.altitude = altitude;
    msg.pitch = pitch;
    msg.roll = roll;
    //msg.heading = heading;
    
    publisher.publish(msg);
}


//main loop
void PositionController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
