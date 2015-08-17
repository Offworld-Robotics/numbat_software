/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "ArduinoConverter.h"
 #include <assert.h>
 #include <ros/ros.h>


int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    ArduinoConverter arduinoConverter;
    arduinoConverter.run();
    
}

ArduinoConverter::ArduinoConverter() {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    fd = fopen(TTY, "w");
    assert(fd != NULL);
    //subscribe to xbox controller
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &ArduinoConverter::joyCallback, this);
    leftDrive = 1500.0;
    rightDrive = 1500.0;       
}

void ArduinoConverter::run() {
    while(ros::ok()) {
        //sendMessage(lfDrive,lmDrive,lbDrive,rfDrive,rmDrive,rbDrive);
        ros::spinOnce();
    }
}


//checks if the button state has changed and changes the feed
void ArduinoConverter::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}

void ArduinoConverter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    #define MAX_IN 1.0
    #define DIFF 0.25

	// Set sensitivity between 0 and 1, 0 makes it output = input, 1 makes output = input ^3
    #define SENSITIVITY 1

    float power = joy->axes[DRIVE_AXES_UD];
    float lr = (-joy->axes[STICK_L_LR]);
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    
    float lDrive  =   (power + lr)/2;
    float rDrive =   -(power - lr)/2;
    
    // The formula in use i: output = (ax^3 + (1-a)x) * 500 + 1500
    // Where a = SENSITIVITY

    leftDrive = ((SENSITIVITY * pow(lDrive, 3) + (1 - SENSITIVITY) * lDrive) * 500) + 1500.0;
    rightDrive = ((SENSITIVITY * pow(rDrive, 3) + (1 - SENSITIVITY) * rDrive) * 500) + 1500.0;
    
}


