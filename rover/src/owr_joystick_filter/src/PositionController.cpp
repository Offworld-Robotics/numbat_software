/*
 * Main class for pbuff relays
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "Bluetongue.h"
#include "ButtonDefs.h"
#include "PositionController.h" 
#include <iostream>
#include <list>
#include <cmath>
#include <stdio.h>

#define TOPIC "/owr/position"
//minum number of lat/long inputs to calculate the heading
#define MIN_H_CALC_BUFFER_SIZE 2 
#define POS_DODGE_TOPIC "/owr/position/dodge"
 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_position_node");
    
    PositionController p(POS_DODGE_TOPIC);
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
    publisher =  node.advertise<owr_messages::position>(topic,10,true);
    gpsSubscriber = node.subscribe("/gps/fix", 1000, &PositionController::receiveGPSMsg, this); // GPS related data
    headingSubscriber = node.subscribe("/owr/heading", 2, &PositionController::receiveHeadingMsg, this);
}

void PositionController::receiveHeadingMsg(const boost::shared_ptr<owr_messages::heading const> & msg) {
    heading = msg->heading;
    sendMsg();
}

void PositionController::receiveGPSMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) {
    //for some reason when the gps dosen't have a fix it gives us values close to zero
    //we want to ignore those
    #define GPS_ERROR_ZONE 1.0 
    //only relay messages with valid lat/long and a good status
    if (!msg->status.status && fabs(msg->latitude) > GPS_ERROR_ZONE) {
        altitude  = msg->altitude;
        latitude  = msg->latitude;
        longitude = msg->longitude;
        std::list<double>::iterator latItr = latitudes.begin();
        double x1 = *(latItr);
        std::list<double>::iterator lonItr = longitudes.begin();
        double y1 = *(lonItr);
        if (x1 != latitude && y1 != longitude) {
            latitudes.push_front(latitude);
            longitudes.push_front(longitude);
        }
    } else {
        ROS_INFO("Dropped invalid gps fix %f, %f, status: %d", 
            msg->latitude, msg->longitude, msg->status.status);
    }
    //updateHeading();
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


void PositionController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    cameraBottomRotateIncRate = 0;
    cameraBottomTiltIncRate = 0;
    cameraTopRotateIncRate = 0;
    cameraTopTiltIncRate = -1;

    // Set sensitivity between 0 and 1, 0 makes it output = input, 1 makes output = input ^3
    if (joy->buttons[BUTTON_A]) {

        msgsOut->axes[CAMERA_BOTTOM_ROTATE] = joy->axes[STICK_L_LR];
        msgsOut->axes[CAMERA_BOTTOM_TILT] = joy->axes[STICK_L_UD];

        // cameraBottomRotateIncRate = joy->axes[STICK_L_LR] * CAMERA_SCALE;
        // cameraBottomTiltIncRate = joy->axes[STICK_L_UD] * CAMERA_SCALE;
    } else if (joy->buttons[BUTTON_B]) {

        msgsOut->axes[CAMERA_TOP_ROTATE] = joy->axes[STICK_L_LR];
        msgsOut->axes[CAMERA_TOP_TILT] = joy->axes[STICK_L_UD];

        // cameraTopRotateIncRate = joy->axes[STICK_L_LR] * CAMERA_SCALE;
        // cameraTopTiltIncRate = joy->axes[STICK_L_UD] * CAMERA_SCALE;
    } else {
        msgsOut->axes[LEFT_WHEELS] = joy->axes[STICK_L_UD];
        msgsOut->axes[RIGHT_WHEELS] = joy->axes[STICK_R_UD];
    }
}

void PositionController::armCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    #define MID_IN 0
    #define DIFF 0.25

    sensor_msgs::Joy msgsOut;
    
    float top = joy->axes[STICK_R_UD] ;//* 0.2;
    msgsOut->axes[ARM_STICK_TOP] = joy->axes[STICK_R_UD];

    float bottom = (joy->axes[STICK_L_UD]) ;//* 0.2;
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    armRotate = joy->axes[STICK_CH_LR];
 
    //armTop = (top / MAX_IN) * 500 + MOTOR_MID  ;
        armIncRate = top * 5;
    

    //TODO: check these actually match up
    armBottom = (bottom / MAX_IN) * 500 + MOTOR_MID  ;

    // Handle claw opening and closing
    if(joy->buttons[BUTTON_LB]) {
        msgsOut->axes[CLAW_STATE] = CLOSE;
    } else if (joy->buttons[BUTTON_RB]) {
        msgsOut->axes[CLAW_STATE] = OPEN;
    } else {
        msgsOut->axes[CLAW_STATE] = STOP;
    }

    //Handle arm rotation
    if(joy->buttons[DPAD_LEFT]) {
        msgsOut->axes[ARM_ROTATE] = ANTICLOCKWISE;
    } else if (joy->buttons[DPAD_RIGHT]) {
        msgsOut->axes[ARM_ROTATE] = CLOCKWISE;
    } else {
        msgsOut->axes[ARM_ROTATE] = STOP;
    }
}


//main loop
void PositionController::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
