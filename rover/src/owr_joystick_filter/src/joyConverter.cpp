/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "JoyConverter.h"
 #include <ros/ros.h>


int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    JoyConverter joyConverter;

    ros::spin();
}

JoyConverter::JoyConverter() {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    
    //uses api at https://github.com/bluesat/owr_software/wiki/OWR-ROS-API
    velPublisher = nh.advertise<geometry_msgs::Twist>("owr/control/drive", 1);
    
    //subscribe to joy stick
    //TODO: at some point we will need to handle two joysticks
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyConverter::joyCallback, this);
    
}

//checks if the button state has changed and changes the feed
void JoyConverter::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}

void JoyConverter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    //direction from Up-down and LR 
    geometry_msgs::Twist vel;
    //TODO: check length to avoid segfaults
    vel.angular.z = joy->axes[DRIVE_AXES_UD];
    vel.linear.x = joy->axes[DRIVE_AXES_LR];
    ROS_INFO("%f", vel.angular.x);
    velPublisher.publish(vel);
    
    //TODO: camera on/off
    //check if the camera button states have changes
    switchFeed(&cam0Button,joy->buttons[CAM_FEED_0],0);
    //TODO: camera rotation
    //TODO: take photo
    //TODO: map zoom in/out
}


