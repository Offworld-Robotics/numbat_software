#include "SensorFusion.h"
#include "ButtonDefs.h"
#include "RoverDefs.h"
#include <cmath>
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#define ACC_WEIGHT 0.05 
#define MAG_WEIGHT 0.0 
#define SYDNEY 12.33
#define POLAND 4.48
#define GYRO_HZ 5
int
main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_sensor_fusion_node");
    
    SensorFusion p;
    p.spin();

    return EXIT_SUCCESS;   
}


SensorFusion::SensorFusion() {
    //set up subs and pub
    allThree = 0;
    subMag = node.subscribe("/mag", 5, &SensorFusion::receiveMag, this);
    
//    subAccel = node.subscribe("/acc", 5, &SensorFusion::receiveAccel, this);
//    subGyro = node.subscribe("/gyro", 5, &SensorFusion::receiveGyro, this);
    pub = node.advertise<owr_messages::heading>("/owr/heading",10);
    
}



void BoardControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
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

void BoardControl::armCallback(const sensor_msgs::Joy::ConstPtr& joy) {

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
