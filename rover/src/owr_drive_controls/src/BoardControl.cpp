/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
#include "BoardControl.h"
#include "Bluetongue.h"
#include <assert.h>
#include <ros/ros.h>

#define MOTOR_MID 1500.0
#define MOTOR_MAX 1900.0
#define MOTOR_MIN 1100.0
#define ROTATION_MID 0.5

#define MAX_IN 1.0
#define DIFF 0.25

// Set sensitivity between 0 and 1, 0 makes it output = input, 1 makes output = input ^3
#define SENSITIVITY 1

static void printStatus(struct status *s) {
	ROS_INFO("Battery voltage: %f", s->batteryVoltage);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    BoardControl BoardControl;
    BoardControl.run();
    
}

BoardControl::BoardControl() {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    //fd = fopen(TTY, "w");
    assert(fd != NULL);

    //subscribe to xbox controller and auton_pathing topics
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &BoardControl::joyCallback, this);
    armSubscriber = nh.subscribe<sensor_msgs::Joy>("arm_joy", 10, &BoardControl::armCallback, this);
    velSubscriber = nh.subscribe<geometry_msgs::Twist>("owr/control/drive", 10, &BoardControl::velCallback, this);

    leftDrive = MOTOR_MID;
    rightDrive = MOTOR_MID; 
    armTop = MOTOR_MID;
    armBottom = MOTOR_MID;
    armRotate = ROTATION_MID;
    armIncRate = 0;
          
}

void BoardControl::run() {
    Bluetongue* steve = new Bluetongue(TTY);

    while(ros::ok()) {
        armTop += armIncRate;
        if (armTop > MOTOR_MAX) {
            armTop = MOTOR_MAX;
        } else if (armTop < MOTOR_MIN) {
            armTop = MOTOR_MIN;
        }
        struct status s = steve->update(leftDrive, rightDrive,
            armTop, armBottom, armRotate);
        //if (s.roverOk == false) {
        //    delete steve;
        //    Bluetongue* steve = new Bluetongue("/dev/ttyACM0");
        //}
	    printStatus(&s);
	    usleep(100000);
        //sendMessage(lfDrive,lmDrive,lbDrive,rfDrive,rmDrive,rbDrive);
        ros::spinOnce();
    }
    delete steve;
}


//checks if the button state has changed and changes the feed
void BoardControl::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}

void BoardControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	// Set sensitivity between 0 and 1, 0 makes it output = input, 1 makes output = input ^3
    leftDrive = (joy->axes[STICK_L_UD]);
    rightDrive = -joy->axes[DRIVE_AXES_UD];
	/*
    float power = joy->axes[DRIVE_AXES_UD];
    float lr = (-joy->axes[STICK_L_LR]);
    
    armRotate = joy->axes[STICK_CH_LR];
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    
    float lDrive  =   ((power + lr)/2)*500 + MOTOR_MID;
    float rDrive =   (-(power - lr)/2)*500 + MOTOR_MID;
    
    // The formula in use i: output = (ax^3 + (1-a)x) * 500 + MOTOR_MID
    // Where a = SENSITIVITY

    //leftDrive = SENSITIVITY * pow(lDrive, 3) + (1 - SENSITIVITY) * lDrive;
    //rightDrive = SENSITIVITY * pow(rDrive, 3) + (1 - SENSITIVITY) * rDrive;
    */
}

void BoardControl::armCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    #define MID_IN 0
    
    float top = joy->axes[STICK_R_UD] ;//* 0.2;
    float bottom = (joy->axes[STICK_L_UD]) ;//* 0.2;
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    armRotate = joy->axes[STICK_CH_LR];
 
    //armTop = (top / MAX_IN) * 500 + MOTOR_MID  ;
    armIncRate = top * 50;
    //TODO: check these actually match up
    armBottom = (bottom / MAX_IN) * 500 + MOTOR_MID  ;
}

// Convert subscribed Twist input to motor vectors for arduino output
void BoardControl::velCallback(const geometry_msgs::Twist::ConstPtr& vel) {

    float power = vel->linear.x;
    float lr = vel->linear.y;

    float lDrive;
    float rDrive;

    // This set of equations ensure the correct proportional powering of the wheels at varying levels of power and lr

    if(lr < 0){
    	lDrive = power + (2 * lr * power);
    	rDrive = power;
    } else if (lr > 0){
    	lDrive = power;
    	rDrive = power - (2 * lr * power);
    } else {
    	lDrive = power;
    	rDrive = power;
    }

    lDrive = (lDrive * 500) + MOTOR_MID;
    rDrive = (rDrive * 500) + MOTOR_MID;

    // The formula in use i: output = (ax^3 + (1-a)x) * 500 + 1500
    // Where a = SENSITIVITY

    leftDrive = (SENSITIVITY * pow(lDrive, 3) + (1 - SENSITIVITY) * lDrive);
    rightDrive = (SENSITIVITY * pow(rDrive, 3) + (1 - SENSITIVITY) * rDrive);
}
