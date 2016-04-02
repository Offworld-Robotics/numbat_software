/*
 * Filters the Joysticks
 * Original Author: Sam S
 * Editors: Harry J.E Day
 * ROS_NODE:
 * ros package: 
 */
 

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "ButtonDefs.h"
#include "RoverDefs.h"

#include "JoystickFilter.h" 

#include <iostream>
#include <list>
#include <cmath>
#include <stdio.h>

#include <geometry_msgs/Twist.h>



#define TOPIC "/owr/position"
//minum number of lat/long inputs to calculate the heading
#define MIN_H_CALC_BUFFER_SIZE 2 
#define JOYSTICK_TOPIC "/owr/joysticks"
#define MID_IN 0
#define DIFF 0.25
#define MAX_IN 1.0
#define MOTOR_MID 1500.0
#define MOTOR_MAX 1900.0
#define MOTOR_MIN 1100.0
#define ROTATION_MID 0.5
#define SENSITIVITY 1 //CANNOT BE 0
#define SPEED_CAP 0.83333 // 3 km/h in m/s
 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_position_node");
    
    JoystickFilter p(JOYSTICK_TOPIC);
    p.spin();
    
    return EXIT_SUCCESS;   
}

JoystickFilter::JoystickFilter(const std::string topic) {
    altitude = 0;
    latitude = 0;
    longitude = 0;
    pitch = 0;
    roll = 0;
    heading = 0;


    //publisher =  node.advertise<owr_messages::position>(topic,10,true);
    publisher = node.advertise<sensor_msgs::Joy>(topic,2,true);
    velPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);
    
    //msgsOut.axes = std::vector<float>(20);
    msgsOut.axes.resize(20);
    msgsOut.buttons.resize(2);
    for(int i = 0; i < msgsOut.buttons.size(); i++) {
        msgsOut.buttons[i] = 0;
    }
    joySubscriber = node.subscribe<sensor_msgs::Joy>("joy",2, &JoystickFilter::joyCallback, this);
    armSubscriber = node.subscribe<sensor_msgs::Joy>("arm_joy", 2, &JoystickFilter::armCallback, this);

}


// Attempt at single joystick driving
void JoystickFilter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    
    geometry_msgs::Twist cmdVel;
    
    float leftWheelSpeed = 0;
    float rightWheelSpeed = 0;
    float leftRightMagnitude = std::abs(joy->axes[STICK_R_LR]/(SENSITIVITY));

    // Set sensitivity between 0 and 1: 
    //  - 0 makes it output = input 
    //  - 1 makes output = input ^3

    if (joy->buttons[BUTTON_A]) {

        msgsOut.axes[CAMERA_BOTTOM_ROTATE] = joy->axes[STICK_L_LR];
        msgsOut.axes[CAMERA_BOTTOM_TILT] = joy->axes[STICK_L_UD];

        // cameraBottomRotateIncRate = joy->axes[STICK_L_LR] * CAMERA_SCALE;
        // cameraBottomTiltIncRate = joy->axes[STICK_L_UD] * CAMERA_SCALE;
    } else if (joy->buttons[BUTTON_B]) {

        msgsOut.axes[CAMERA_TOP_ROTATE] = joy->axes[STICK_L_LR];
        msgsOut.axes[CAMERA_TOP_TILT] = joy->axes[STICK_L_UD];

        // cameraTopRotateIncRate = joy->axes[STICK_L_LR] * CAMERA_SCALE;
        // cameraTopTiltIncRate = joy->axes[STICK_L_UD] * CAMERA_SCALE;
    } else {

//         if(STICK_R_LR > 0){
// 
//             leftWheelSpeed = joy->axes[STICK_R_UD] + leftRightMagnitude;
//             rightWheelSpeed = joy->axes[STICK_R_UD] - leftRightMagnitude;    
// 
//         } else if(STICK_R_LR < 0){
// 
//             leftWheelSpeed = joy->axes[STICK_R_UD] - leftRightMagnitude;
//             rightWheelSpeed = joy->axes[STICK_R_UD] + leftRightMagnitude;    
// 
//         } else {
// 
//             // Minus value of LR to get the total value back between {-1..1}
//             leftWheelSpeed = joy->axes[STICK_R_UD] - leftRightMagnitude;
//             rightWheelSpeed = joy->axes[STICK_R_UD] - leftRightMagnitude;
// 
//             // //Divide by 2 so that the max value for left/rightWheelSpeed can never exceed {-1..1} 
//             // leftWheelSpeed = joy->axes[STICK_R_UD]/2;
//             // rightWheelSpeed = joy->axes[STICK_R_UD]/2;
// 
//         }
        //left stick controls magnitude
        //right stick controls direction
        //joystick values are between 1 and -1
        double magnitude = joy->axes[SPEED_STICK] * SPEED_CAP;
        cmdVel.linear.x = joy->axes[DIRECTION_STICK_X] * magnitude;
        cmdVel.linear.y = joy->axes[DIRECTION_STICK_Y] * magnitude * -1;

    }
    msgsOut.buttons[FL_SWERVE_RESET] = joy->buttons[BUTTON_STICK_L];
    msgsOut.buttons[FR_SWERVE_RESET] = joy->buttons[BUTTON_STICK_R];
    msgsOut.axes[LEFT_WHEELS] = leftWheelSpeed;
    msgsOut.axes[RIGHT_WHEELS] = rightWheelSpeed;
    
    publisher.publish(msgsOut);
    velPublisher.publish(cmdVel);
}




void JoystickFilter::armCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    // Handle arm movement
    msgsOut.axes[ARM_STICK_TOP] = joy->axes[STICK_R_UD];
    msgsOut.axes[ARM_STICK_BOTTOM] = (joy->axes[STICK_L_UD]) ;//* 0.2;
    msgsOut.axes[ARM_ROTATE] = joy->axes[STICK_CH_LR];
    
    // Handle claw opening and closing
    if(joy->buttons[BUTTON_LB]) {
        msgsOut.axes[CLAW_STATE] = CLOSE;
    } else if (joy->buttons[BUTTON_RB]) {
        msgsOut.axes[CLAW_STATE] = OPEN;
    } else {
        msgsOut.axes[CLAW_STATE] = STOP;
    }

    //Handle claw rotation
    if(joy->buttons[BUTTON_A]){
        msgsOut.axes[CLAW_ROTATE] = ANTICLOCKWISE;
    } else if(joy->buttons[BUTTON_B]){
        msgsOut.axes[CLAW_ROTATE] = CLOCKWISE;
    } else {
        msgsOut.axes[CLAW_ROTATE] = STOP;
    }

    // //Handle arm rotation
    // if(joy->buttons[DPAD_LEFT]) {
    //     msgsOut.axes[ARM_ROTATE] = ANTICLOCKWISE;
    // } else if (joy->buttons[DPAD_RIGHT]) {
    //     msgsOut.axes[ARM_ROTATE] = CLOCKWISE;
    // } else {
    //     msgsOut.axes[ARM_ROTATE] = STOP;
    // }
    // ROS_INFO("HERE: %d", ARM_ROTATE);
    // ROS_INFO("AND HERE: %f", msgsOut.axes[ARM_ROTATE]);

    publisher.publish(msgsOut);
}


//main loop
void JoystickFilter::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}
