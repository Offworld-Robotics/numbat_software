/*
 * Filters the Joysticks
 * Original Author: Sam S
 * Editors: Harry J.E Day, Sean Thompson
 * ROS_NODE:owr_joystick_filter
 * ros package:
 */


// #include "bluesat_owr_protobuf/Message1Relay.h"
#include "ButtonDefs.h"
#include "RoverDefs.h"
#include "JoystickFilter.h"

#include <iostream>
#include <list>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdio.h>

#include <geometry_msgs/Twist.h>

#define TOPIC "/owr/position"
// minum number of lat/long inputs to calculate the heading
#define MIN_H_CALC_BUFFER_SIZE 2
#define JOYSTICK_TOPIC "/owr/joysticks"
#define MID_IN 0
#define DIFF 0.25
#define MAX_IN 1.0
#define MOTOR_MID 1500.0
#define MOTOR_MAX 1900.0
#define MOTOR_MIN 1100.0
#define ROTATION_MID 0.5
#define SENSITIVITY 1  // CANNOT BE 0
#define SPEED_CAP 0.83333  // 3 km/h in m/s

// DEADZONES for thumbsticks, these will require tuning through testing
#define DEADZONE_RADIAL_STICK_L 0.1
#define DEADZONE_RADIAL_STICK_R 0.1

#define LIDAR_CONTINUOS 1
#define LIDAR_STATIONARY 2
#define LIDAR_POSITION 3
#define LIDAR_MULTIPLIER 0.01

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
    publisher = node.advertise<sensor_msgs::Joy>(topic,2,true);
    velPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel",1,false);
    unfilteredVelPublisher = node.advertise<geometry_msgs::Twist>("/unscaled_vector",1,false);

    lidarModePublisher = node.advertise<std_msgs::Int16>("/owr/lidar_gimble_mode", 1, true);
    lidarPosPublisher = node.advertise<std_msgs::Float64>("/laser_tilt_joint_controller/command",1,true);

    //msgsOut.axes = std::vector<float>(20);
    msgsOut.axes.resize(20);
    msgsOut.buttons.resize(2);
    for(int i = 0; i < msgsOut.buttons.size(); i++) {
        msgsOut.buttons[i] = 0;
    }
    joySubscriber = node.subscribe<sensor_msgs::Joy>("joy",2, &JoystickFilter::joyCallback, this);
    armSubscriber = node.subscribe<sensor_msgs::Joy>("arm_joy", 2, &JoystickFilter::armCallback, this);

}


// Take Input from the GamePad ie Xbox Controller
void JoystickFilter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    geometry_msgs::Twist cmdVel, unfiltVel;

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
    gimbalRate = 0.0;
    //IMPORTANT: this will do nothing if the lidar is not is position mode
    gimbalRate = joy->axes[STICK_CH_UD];
    if (joy->buttons[BUTTON_A]) {

        // Thumb sticks control camera rotation while A Button is held

        msgsOut.axes[CAMERA_BOTTOM_ROTATE] = joy->axes[STICK_L_LR];
        msgsOut.axes[CAMERA_BOTTOM_TILT] = joy->axes[STICK_L_UD];

        // cameraBottomRotateIncRate = joy->axes[STICK_L_LR] * CAMERA_SCALE;
        // cameraBottomTiltIncRate = joy->axes[STICK_L_UD] * CAMERA_SCALE;
    } else if (joy->buttons[BUTTON_B]) {
        // Thumb sticks control top camera rotation while B Button is held

        msgsOut.axes[CAMERA_TOP_ROTATE] = joy->axes[STICK_L_LR];
        msgsOut.axes[CAMERA_TOP_TILT] = joy->axes[STICK_L_UD];

        // cameraTopRotateIncRate = joy->axes[STICK_L_LR] * CAMERA_SCALE;
        // cameraTopTiltIncRate = joy->axes[STICK_L_UD] * CAMERA_SCALE;
    } else {
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

        double signXAxis = (rescaledLStick(0) > 0.0) ? 1.0: -1.0;
        Eigen::Vector2d unfiltVector(0.0, 1 * signXAxis);
        unfiltVector = Eigen::Rotation2D<double>(M_PI/2 * pow(rescaledRStick(0), 2)*signYAxis) * unfiltVector;


        unfiltVel.linear.x = unfiltVector(1);
        unfiltVel.linear.y = unfiltVector(0);
        unfiltVel.linear.z = rescaledLStick(1);


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
    }
    msgsOut.buttons[FL_SWERVE_RESET] = joy->buttons[BUTTON_STICK_L];
    msgsOut.buttons[FR_SWERVE_RESET] = joy->buttons[BUTTON_STICK_R];
    msgsOut.axes[LEFT_WHEELS] = leftWheelSpeed;
    msgsOut.axes[RIGHT_WHEELS] = rightWheelSpeed;

    publisher.publish(msgsOut);
    velPublisher.publish(cmdVel);
    unfilteredVelPublisher.publish(unfiltVel);
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
    ros::Rate r(20);
    while(ros::ok()) {

        lidarPos.data += gimbalRate * LIDAR_MULTIPLIER;
        lidarPosPublisher.publish<std_msgs::Float64>(lidarPos);

        r.sleep();
        ros::spinOnce();
    }
}
