/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
#include "BoardControl.h"
#include "Bluetongue.h"
#include <assert.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>

#define MOTOR_MID 1500.0
#define MOTOR_MAX 1900.0
#define MOTOR_MIN 1100.0
#define ROTATION_MID 0.5

#define CLAW_ROTATION_MID 45
#define CLAW_ROTATION_MAX 90
#define CLAW_ROTATION_MIN 0

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
    //assert(fd != NULL);
    //subscribe to xbox controller
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy",2, &BoardControl::joyCallback, this, transportHints);
    armSubscriber = nh.subscribe<sensor_msgs::Joy>("arm_joy", 2, &BoardControl::armCallback, this,transportHints);
    gpsPublisher = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix",  10);
    magPublisher = nh.advertise<geometry_msgs::Vector3>("mag", 10);
    velSubscriber = nh.subscribe<geometry_msgs::Twist>("/owr/auton_twist", 2, &BoardControl::velCallback, this, transportHints);
    leftDrive = MOTOR_MID;
    rightDrive = MOTOR_MID; 
    armTop = MOTOR_MID;
    armBottom = MOTOR_MID;
    armRotate = ROTATION_MID;
    clawRotate = CLAW_ROTATION_MID;
    clawGrip = CLAW_ROTATION_MID;
    armIncRate = 0;
    gpsSequenceNum = 0;
    rotState = STOP;
    clawState = STOP;
          
}

void BoardControl::run() {
    std::string board;
    nh.param<std::string>("board_tty", board, TTY);
    ROS_INFO("connecting to board on %s", board.c_str());
    Bluetongue* steve = new Bluetongue(board.c_str());

    while(ros::ok()) {
        armTop += armIncRate;
        if (armTop > MOTOR_MAX) {
            armTop = MOTOR_MAX;
        } else if (armTop < MOTOR_MIN) {
            armTop = MOTOR_MIN;
        }
        
        
        if (clawState == OPEN) {
            clawRotate += 5;
        } else if (clawState == CLOSE) {
            clawRotate-= 5;
        }
        
        
        if (clawRotate > CLAW_ROTATION_MAX) {
            clawRotate = CLAW_ROTATION_MAX;
        } else if (clawRotate < CLAW_ROTATION_MIN) {
            clawRotate = CLAW_ROTATION_MIN;
        }
        
        
        if (clawGrip == OPEN) {
            clawGrip += 5;
        } else if (clawGrip == CLOSE) {
            clawGrip -=  5;
        }
        
        if (clawGrip > CLAW_ROTATION_MAX) {
            clawGrip = CLAW_ROTATION_MAX;
        } else if (clawGrip < CLAW_ROTATION_MIN) {
            clawGrip = CLAW_ROTATION_MIN;
        }
        
        struct status s = steve->update(leftDrive, rightDrive,
            armTop, armBottom, armRotate, ((float)clawRotate/(float)CLAW_ROTATION_MAX)*1000.0+1000, ((float)clawGrip/(float)CLAW_ROTATION_MAX)*1000.0+1000);

        publishGPS(s.gpsData);
        publishMag(s.magData);
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

void BoardControl::publishGPS(GPSData gps) {
    sensor_msgs::NavSatFix msg;
    msg.longitude = ((float)gps.longitude)/GPS_FLOAT_OFFSET;
    msg.latitude = ((float)gps.latitude)/GPS_FLOAT_OFFSET;
    msg.altitude = gps.altitude;
    
    if (gps.fixValid) {
        msg.status.status = msg.status.STATUS_FIX;
    } else {
        msg.status.status = msg.status.STATUS_NO_FIX;
    }
    msg.status.service = msg.status.SERVICE_GPS; //NOt sure this is right
    msg.header.seq = gpsSequenceNum;
    msg.header.frame_id = 1; // global frame
    gpsPublisher.publish(msg);
}

void BoardControl::publishMag(MagData mag) {
    geometry_msgs::Vector3 msg;
    msg.x = mag.x;
    msg.y = mag.y;
    msg.z = mag.z;
    // Header just has dummy values
    magPublisher.publish(msg);
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
    #define DIFF 0.25
    
    float top = joy->axes[STICK_R_UD] ;//* 0.2;
    float bottom = (joy->axes[STICK_L_UD]) ;//* 0.2;
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    armRotate = joy->axes[STICK_CH_LR];
 
    //armTop = (top / MAX_IN) * 500 + MOTOR_MID  ;
    armIncRate = top * 50;
    //TODO: check these actually match up
    armBottom = (bottom / MAX_IN) * 500 + MOTOR_MID  ;
    
    if(joy->buttons[BUTTON_LB]) {
        clawState = OPEN;
    } else if (joy->buttons[BUTTON_RB]) {
        clawState = CLOSE;
    } else {
        clawState = STOP;
    }

    if(joy->buttons[BUTTON_A]) {
        rotState = OPEN;
    } else if (joy->buttons[BUTTON_B]) {
        rotState = CLOSE;
    } else {
        rotState = STOP;
    }
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
    
    lDrive = power;
    rDrive = power;
    leftDrive = lDrive;
    rightDrive = rDrive;
    //leftDrive = (lDrive * 400) + MOTOR_MID;
    //rightDrive = (rDrive * 400) + MOTOR_MID;
    ROS_ERROR("%f,%f", leftDrive, rightDrive);

    // The formula in use i: output = (ax^3 + (1-a)x) * 500 + 1500
    // Where a = SENSITIVITY

    //leftDrive = (SENSITIVITY * pow(lDrive, 3) + (1 - SENSITIVITY) * lDrive);
    //rightDrive = (SENSITIVITY * pow(rDrive, 3) + (1 - SENSITIVITY) * rDrive);
}
