/*
 * Filters the Joysticks
 * Original Author: Sam S
 * Editors: Harry J.E Day, Sean Thompson, Elliott Smith
 * ROS_NODE:owr_joystick_filter
 * ros package: 
 */
     

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "ButtonDefs.h"
#include "RoverDefs.h"
#include "JoystickFilter.h" 
//#include "Bluetounge2Params.h" TODO: Fix this later

#include <iostream>
#include <list>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
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

// DEADZONES for thumbsticks, these will require tuning through testing
#define DEADZONE_RADIAL_STICK_L 0.1
#define DEADZONE_RADIAL_STICK_R 0.1

#define LIDAR_CONTINUOS 1
#define LIDAR_STATIONARY 2
#define LIDAR_POSITION 3
#define LIDAR_MULTIPLIER 0.01

// Claw constants
#define CLAW_ROTATION_MAX 90
#define CLAW_ROTATION_MID 45


#define CLAW_STOP_PWN 1500
#define CLAW_CLOCKWISE_PWM 1490
#define CLAW_ANTICLOCKWISE_PWM 1510

// Arm constants
#define ARM_ROTATE_RATE 0.01

 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_position_node");
    
    JoystickFilter p(JOYSTICK_TOPIC);
    p.spin();
    
    return EXIT_SUCCESS;   
}

JoystickFilter::JoystickFilter(const std::string topic) :
    gimbalRate(0.0) {
    altitude = 0;
    latitude = 0;
    longitude = 0;
    pitch = 0;
    roll = 0;
    heading = 0;
    
    lidarModeMsg.data = LIDAR_CONTINUOS;


    //publisher =  node.advertise<owr_messages::position>(topic,10,true);
    publisher = node.advertise<sensor_msgs::Joy>(topic,2,true); //nothing should go through this
    
    velPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel",1,false);
    
    armUpperActPub = node.advertise<std_msgs::Float64>("/upper_arm_act_controller/command",2,false); // TODO: WHAT SHOULD THE REMAINING PARAMETERS AFTER THE TOPIC BE?
    armLowerActPub = node.advertise<std_msgs::Float64>("/lower_arm_act_controller/command",2,false);
    armBaseRotatePub = node.advertise<std_msgs::Float64>("/arm_base_rotate_controller/command",2,false);
    clawRotateRub = node.advertise<std_msgs::Float64>("/claw_rotate_controller/command",2,false);
    clawGripPub = node.advertise<std_msgs::Float64>("/claw_grip_controller/command",2,false);
    
    lidarModePublisher = node.advertise<std_msgs::Int16>("/owr/lidar_gimble_mode", 1, true);
    lidarPosPublisher = node.advertise<std_msgs::Float64>("/laser_tilt_joint_controller/command",1,true);
    

    joySubscriber = node.subscribe<sensor_msgs::Joy>("joy",2, &JoystickFilter::joyCallback, this);
    armSubscriber = node.subscribe<sensor_msgs::Joy>("arm_joy", 2, &JoystickFilter::armCallback, this);

}


// Take Input from the GamePad ie Xbox Controller
void JoystickFilter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    
    geometry_msgs::Twist cmdVel;

    // Implement a Scaled Radial Deadzone as in 
    // http://www.third-helix.com/2013/04/12/doing-thumbstick-dead-zones-right.html 

    // Left Thumbstick
    double L_LR = joy->axes[STICK_L_LR];
    double L_UD = joy->axes[STICK_L_UD];
    //ROS_INFO("joy->axes[STICK_L_LR]:%f,joy->axes[STICK_L_UD]:%f", L_LR, L_UD);
    rawLStick = Eigen::Vector2d(L_LR, L_UD);
    //rawLStick(joy->axes[STICK_L_LR], joy->axes[STICK_L_UD]*-1.0);
    //ROS_INFO("rawLStick 0:%f,1:%f", rawLStick(0), rawLStick(1));
    rawMagLStick = fmin(1.0, rawLStick.norm());
    //ROS_INFO("rawMagLStick:%f ie output of rawLStick.norm()", rawMagLStick);
    if(rawMagLStick > 0.0){
        // need to apply normalisation
        deadZoneCorrectedMagL = fmax(0.0, (rawMagLStick - DEADZONE_RADIAL_STICK_L));
        //ROS_INFO("deadZoneCorrectedMagL:%f", deadZoneCorrectedMagL);
        deadzoneRescaledLStickMag = (deadZoneCorrectedMagL)/(1.0 - (DEADZONE_RADIAL_STICK_L));
        //ROS_INFO("deadzoneRescaledLStickMag:%f", deadzoneRescaledLStickMag);
        rescaledLStick = rawLStick;
        rescaledLStick.normalize();
        //ROS_INFO("rescaledLStick 0:%f,1:%f", rescaledLStick(0), rescaledLStick(1));
        rescaledLStick *= deadzoneRescaledLStickMag;
    } else {
        // rawMagLStick == 0.0, no stick deflection cannot normalise a zero vector
        rescaledLStick = Eigen::Vector2d(0.0, 0.0);
    }
    //ROS_INFO("rescaledLStick 0:%f,1:%f", rescaledLStick(0), rescaledLStick(1));
    
    // Right Thumbstick
    // Multiply y axis value of joystick by -1.0 to make posive up
    rawRStick = Eigen::Vector2d(joy->axes[STICK_R_LR], joy->axes[STICK_R_UD]);
    rawMagRStick = fmin(1.0, rawRStick.norm());
    if(rawMagRStick > 0.0){
        deadZoneCorrectedMagR = fmax(0.0, (rawMagRStick - DEADZONE_RADIAL_STICK_R));
        deadzoneRescaledRStickMag = (deadZoneCorrectedMagR)/(1.0 - (DEADZONE_RADIAL_STICK_R));
        rescaledRStick = rawRStick;
        rescaledRStick.normalize();
        rescaledRStick *= deadzoneRescaledRStickMag;
    } else {
        // rawMagRStick == 0.0, no stick deflection cannot normalise a zero vector
        rescaledRStick = Eigen::Vector2d(0.0, 0.0);
    }
    //ROS_INFO("rescaledRStick 0:%f,1:%f", rescaledRStick(0), rescaledRStick(1));
    
    float leftWheelSpeed = 0;
    float rightWheelSpeed = 0;
    //float leftRightMagnitude = std::abs(joy->axes[STICK_R_LR]/(SENSITIVITY));
    
    /*
     * Toggle lidar mode
     */
     if(joy->buttons[BUTTON_LB]) {
         lidarModeMsg.data = (lidarModeMsg.data + 1) % 3;
         lidarModePublisher.publish<std_msgs::Int16>(lidarModeMsg);
     } else if(joy->buttons[BUTTON_RB]) {
         lidarModeMsg.data =abs((lidarModeMsg.data - 1) % 3);
         lidarModePublisher.publish<std_msgs::Int16>(lidarModeMsg);
     }

    /*
     * Control Modes
     *
     * Hold Button A
     *      Thumb Sticks control lidar position
     *
     *  Hold Button B
     *      Thumb Sticks control top camera rotation
     *
     *  Normal Driving Mode
     *      Active by default
     *      
     */
     // New Steering Scheme Implementation
     // left stick is forwards backwards (only the vertical axis is used)
     // right stick is steering angle (only the horizontal axis is used)
        
     // Note coordinate system of cmdVel.linear message used to communicat with the board
     // x = forward-backwards  forwards positive
     // y = port-starboard 
        
     // Note Eigen::Vector2d storing axes values as (x,y) 
     //  therefore index 0 == x, index 1 == y
     // set cmdVel.linear.y  = x-axis of R stick rescaled input
     // get the sign (positive or negative of the axis)
     double signYAxis = (rescaledRStick(0) > 0.0) ? 1.0: -1.0;
     //  Magnitude of driveVector is set by rescaledLStick(1);
     Eigen::Vector2d driveVector(0.0, rescaledLStick(1));
     //  map direction vector argument ie angle to 
     //  apply squared scaling of the stick input then reset the sign to give
     //      lower sensitivity for small steering angles
     driveVector = Eigen::Rotation2D<double>(M_PI/2 * pow(rescaledRStick(0), 2)*signYAxis) * driveVector;

     ROS_INFO("\eRstick(0):%f Rstick(1):%f", rescaledRStick(0),rescaledRStick(1));
     ROS_INFO("\eLstick(0):%f Lstick(1):%f", rescaledLStick(0),rescaledLStick(1));
     ROS_INFO("\nrotated driveVector (0):%f (1):%f", driveVector(0),driveVector(1));
     cmdVel.linear.x = driveVector(1);
     cmdVel.linear.y = driveVector(0);
     ROS_INFO("\nNEW Sticks cmdVel.linear Y:%f X:%f", cmdVel.linear.y, cmdVel.linear.x);


     /*
     //OLD IMPLEMENTATION, MAGNITUDE STICK & DIRECTION STICK BOTH LINEAR WITH NO DEADZONE 
     //left stick controls magnitude
     //right stick directly chooses direction vector
     //  direction in 2d & magnitude by stick deflection
     //joystick values are between 1 and -1
     double magnitude = joy->axes[SPEED_STICK] * SPEED_CAP;
     // Add Cubic scaling of direction stick input
     //  this will reduce sensitivity a lot for small deflections,
     //      and smoothly increase in sensitivity for larger deflections.
     cmdVel.linear.x = pow(joy->axes[DIRECTION_STICK_X],3) * magnitude;
     cmdVel.linear.y = pow(joy->axes[DIRECTION_STICK_Y],3) * magnitude * -1;
     //ROS_INFO("OLD STICKS  cmdVel.linear X:%f Y:%f \n", cmdVel.linear.x, cmdVel.linear.y);
     */

    velPublisher.publish(cmdVel);
}




void JoystickFilter::armCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    // Handle arm movement
    float armActTop = joy->axes[STICK_R_UD];
    float armActBottom = (joy->axes[STICK_L_UD]) ;//* 0.2;
    
    // Convert to PWM
    float armBottom = (armActBottom / MAX_IN) * 500 + MOTOR_MID  ;
    float armTop = (armActTop / MAX_IN) * 500 + MOTOR_MID  ;
    
    
    clawState = STOP; //default is stop
    // Handle claw opening and closing by writing state to class variable clawState
    if(joy->buttons[BUTTON_LB]) {
        clawState = CLOSE;
    } else if (joy->buttons[BUTTON_RB]) {
        clawState = OPEN;
    } 
    
    // This also isn't working WHY?
    float clawRotatePWM = CLAW_STOP_PWN; // default is stop
    //Handle claw rotation
    if(joy->buttons[BUTTON_A]){
        clawRotatePWM = CLAW_ANTICLOCKWISE_PWM;
    } else if(joy->buttons[BUTTON_B]){
        clawRotatePWM = CLAW_CLOCKWISE_PWM;
    } 
    
    
    // The DPAD needs to be treated as a joystick value between -1 and 1
    // HOW do i read this value ??
    float armRotate = STOP;
    //Handle arm rotation
    if(joy->axes[DPAD_LR]<0) {
        armRotate = ANTICLOCKWISE;
    } else if (joy->buttons[DPAD_LR]>0) {
        armRotate = CLOCKWISE;
    } 
    
    float armRotatePWM = armRotate*ARM_ROTATE_RATE;

   
    
    // create the messages and set the data field
      
    std_msgs::Float64 armUpperActMessage;
    armUpperActMessage.data = armTop;
    
    std_msgs::Float64 armLowerActMessage;
    armLowerActMessage.data = armBottom;
    
    std_msgs::Float64 armRotateMessage;
    armRotateMessage.data = armRotatePWM;
    
    std_msgs::Float64 clawRotateMessage;
    clawRotateMessage.data = clawRotatePWM;
  
    
    // publish messages to arm topics
    armUpperActPub.publish(armUpperActMessage);
    armLowerActPub.publish(armLowerActMessage);
    armBaseRotatePub.publish(armRotateMessage);
    clawRotateRub.publish(clawRotateMessage);    
    
    // Claw grip publishing done in spin() function [LOOP]
    
}


//main loop
void JoystickFilter::spin() {
    ros::Rate r(20);
    float clawGrip = CLAW_ROTATION_MID;
    while(ros::ok()) {
        
        lidarPos.data += gimbalRate * LIDAR_MULTIPLIER;
        lidarPosPublisher.publish<std_msgs::Float64>(lidarPos);
	if (clawState == OPEN) {
	  clawGrip += 1;
	} else if (clawState == CLOSE) {
	  clawGrip -= 1;
	}
	
	float clawGripPWM = (clawGrip/(float)CLAW_ROTATION_MAX)*1000.0 + 1000;  
        std_msgs::Float64 clawGripMessage;
	clawGripMessage.data = clawGripPWM;
	
	clawGripPub.publish(clawGripMessage);
	
        r.sleep();
        ros::spinOnce();
    }
}
