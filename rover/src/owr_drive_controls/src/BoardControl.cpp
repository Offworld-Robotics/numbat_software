/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
#include "BoardControl.h"
#include "Bluetongue.h"
#include "RoverDefs.h"
#include "ButtonDefs.h"
#include <assert.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#define MOTOR_MID 1500.0
#define MOTOR_MAX 1900.0
#define MOTOR_MIN 1100.0
#define ROTATION_MID 0.5

#define CLAW_ROTATION_MID 45
#define CLAW_ROTATION_MAX 90
#define CLAW_ROTATION_MIN 0

#define CAMERA_ROTATION_MID 90
#define CAMERA_ROTATION_MAX 180
#define CAMERA_ROTATION_MIN 0
#define CAMERA_SCALE 1

#define MAX_IN 1.0
#define DIFF 0.25

#define RECONNECT_DELAY 1000

#define WHEEL_MOTOR_MAX_PWM 2000
#define WHEEL_MOTOR_MIN_PWM 1000
#define WHEEL_MOTOR_RPM 24000 //TODO: confirm this
#define WHEEL_RADIUS 1.0 //TODO: this is made up!

//this one has to be const because array
const double SWERVE_GEARS[] = {0.1, 0.2}; //TODO: put in not stupid values here
#define SWERVE_N_GEARS 2
#define SWERVE_MOTOR_MAX_PWM 2000
#define SWERVE_MOTOR_MIN_PWM 1000
#define SWERVE_MOTOR_RPM 24 //TODO: check this
#define SWERVE_RADIUS 0.01 //TODO: get this

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

BoardControl::BoardControl() : 
    jMonitor(nh),
    frontLeftWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        "/front_left_wheel_axel_controller/command",
        nh,
        "front_left_wheel_axel"
    ),
    frontRightWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        "/front_right_wheel_axel_controller/command",
        nh,
        "front_right_wheel_axel"
    ), backLeftWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        "/back_left_wheel_axel_controller/command",
        nh,
        "back_left_wheel_axel"
    ),
    backRightWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        "/back_right_wheel_axel_controller/command",
        nh,
        "back_right_wheel_axel"
    ),
    frontLeftSwerve(
        SWERVE_RADIUS,
        SWERVE_GEARS,
        SWERVE_N_GEARS,
        SWERVE_MOTOR_MIN_PWM,
        SWERVE_MOTOR_MAX_PWM,
        SWERVE_MOTOR_RPM,
        "/front_left_wheel_axel_controller/command",
        nh,
        "front_left_wheel_axel"
    ),
    frontRightSwerve(
       SWERVE_RADIUS,
        SWERVE_GEARS,
        SWERVE_N_GEARS,
        SWERVE_MOTOR_MIN_PWM,
        SWERVE_MOTOR_MAX_PWM,
        SWERVE_MOTOR_RPM,
        "/front_right_swerve_controller/command",
        nh,
        "front_right_swerve"
    ), backLeftSwerve(
        SWERVE_RADIUS,
        SWERVE_GEARS,
        SWERVE_N_GEARS,
        SWERVE_MOTOR_MIN_PWM,
        SWERVE_MOTOR_MAX_PWM,
        SWERVE_MOTOR_RPM,
        "/back_left_swerve_controller/command",
        nh,
        "back_left_swerve"
    ),
    backRightSwerve(
        SWERVE_RADIUS,
        SWERVE_GEARS,
        SWERVE_N_GEARS,
        SWERVE_MOTOR_MIN_PWM,
        SWERVE_MOTOR_MAX_PWM,
        SWERVE_MOTOR_RPM,
        "/back_right_swerve_controller/command",
        nh,
        "back_right_swerve"
    ) {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    //fd = fopen(TTY, "w");
    //assert(fd != NULL);
    //subscribe to xbox controller
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("/owr/joysticks",2, &BoardControl::controllerCallback, this, transportHints);
//    armSubscriber = nh.subscribe<sensor_msgs::Joy>("arm_joy", 2, &BoardControl::armCallback, this,transportHints);
    gpsPublisher = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix",  10);
    magPublisher = nh.advertise<geometry_msgs::Vector3>("mag", 10);
    gyroPublisher = nh.advertise<geometry_msgs::Vector3>("gyro", 10);
    accPublisher = nh.advertise<geometry_msgs::Vector3>("acc", 10);
    battVoltPublisher = nh.advertise<std_msgs::Float64>("battery_voltage", 10);
    voltmeterPublisher = nh.advertise<std_msgs::Float64>("voltmeter", 10);
    boardStatusPublisher = nh.advertise<owr_messages::board>("/owr/board_status",10);
    velSubscriber = nh.subscribe<geometry_msgs::Twist>("/owr/auton_twist", 2, &BoardControl::velCallback, this, transportHints);
    leftDrive = MOTOR_MID;
    rightDrive = MOTOR_MID; 
    armTop = MOTOR_MID;
    armBottom = MOTOR_MID;
    armRotate = ROTATION_MID;
    clawRotate = CLAW_ROTATION_MID;
    clawGrip = CLAW_ROTATION_MID;
    cameraBottomRotate = CAMERA_ROTATION_MID;
    cameraBottomTilt = CAMERA_ROTATION_MID;
    cameraTopRotate = CAMERA_ROTATION_MID;
    cameraTopTilt = CAMERA_ROTATION_MID;
    armIncRate = 0;
    cameraBottomRotateIncRate = 0;
    cameraBottomTiltIncRate = 0;
    cameraTopRotateIncRate = 0;
    cameraTopTiltIncRate = 0;
    gpsSequenceNum = 0;
    rotState = STOP;
    clawState = STOP;
    
    

}

int clawRotScale(int raw) {
    return ((float)raw/(float)CLAW_ROTATION_MAX)*1000.0 + 1000;
}
int cameraRotScale(int raw) {
    return ((float)raw/(float)CAMERA_ROTATION_MAX)*1000.0 + 1000;
}


void cap(int *a, int low, int high) {
    *a = *a < low ? low : *a;
    *a = *a > high ? high : *a;
}

void BoardControl::run() {
    std::string board;
    nh.param<std::string>("board_tty", board, TTY);
    ROS_INFO("connecting to board on %s", board.c_str());
    Bluetongue* steve = new Bluetongue(board.c_str());
    struct status s;
    s.isConnected = true;
    ros::Rate r(5);
    int cbr = 0, cbt = 0;
    while (ros::ok()) {
        while(ros::ok()) {
            //cbr = cbr < 120 ? cbr + 5 : 0;
            //cbt = cbt < 70 ? cbt + 5 : 0;
            armTop += armIncRate;
            cap(&armTop, MOTOR_MIN, MOTOR_MAX);
            
            cameraBottomRotate += cameraBottomRotateIncRate;
            cap(&cameraBottomRotate, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);
            
            cameraBottomTilt += cameraBottomTiltIncRate;
            cap(&cameraBottomTilt, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);
            
            cameraTopRotate += cameraTopRotateIncRate;
            cap(&cameraTopRotate, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);
            
            cameraTopTilt += cameraTopTiltIncRate;
            cap(&cameraTopTilt, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);

            if (rotState == OPEN) {
                clawRotate += 5;
            } else if (rotState == CLOSE) {
                clawRotate-= 5;
            }
            cap(&clawRotate, CLAW_ROTATION_MIN, CLAW_ROTATION_MAX);
            
            if (clawState  == OPEN) {
                clawGrip += 5;
            } else if (clawState  == CLOSE) {
                clawGrip -=  5;
            }
            cap(&clawGrip, CLAW_ROTATION_MIN, CLAW_ROTATION_MAX); 
            
            //cameraBottomTilt = cbt;
            //cameraBottomRotate = cbr; 
            struct status s = steve->update(leftDrive, rightDrive,
                armTop, armBottom, armRotate, clawRotScale(clawRotate),
                clawRotScale(clawGrip), cameraRotScale(cameraBottomRotate),
                cameraRotScale(cameraBottomTilt), 
                cameraRotScale(cameraTopRotate), cameraRotScale(cameraTopTilt), 1330); 
            owr_messages::board statusMsg;
            statusMsg.leftDrive = leftDrive;
            statusMsg.rightDrive = rightDrive;
            boardStatusPublisher.publish(statusMsg);
            if (!s.isConnected) break;

            publishGPS(s.gpsData);
            publishMag(s.magData);
            publishIMU(s.imuData);
            publishBattery(s.batteryVoltage);
            //publishVoltmeter(s.voltmeter);
            printStatus(&s);
            //sendMessage(lfDrive,lmDrive,lbDrive,rfDrive,rmDrive,rbDrive);
            ros::spinOnce();
            r.sleep();
        }
        ROS_ERROR("Lost usb connection to Bluetongue");
        ROS_ERROR("Trying to reconnect every %d ms", RECONNECT_DELAY);
        bool success = false;
        while (ros::ok() && !success) {
            usleep(RECONNECT_DELAY * 1000);
            success = steve->reconnect();
            if (success) ROS_INFO("Bluetongue reconnected!");
            else ROS_WARN("Bluetongue reconnection failed. Trying again soon...");
        }
        cap(&clawGrip, CLAW_ROTATION_MIN, CLAW_ROTATION_MAX); 
        
        

        struct status s = steve->update(leftDrive, rightDrive,
            armTop, armBottom, armRotate, clawRotScale(clawRotate),
            clawRotScale(clawGrip), cameraRotScale(cameraBottomRotate),
            cameraRotScale(cameraBottomTilt), 
            cameraRotScale(cameraTopRotate), cameraRotScale(cameraTopTilt), 1330); 

        publishGPS(s.gpsData);
        publishMag(s.magData);
        publishIMU(s.imuData);
        
        printStatus(&s);
        //sendMessage(lfDrive,lmDrive,lbDrive,rfDrive,rmDrive,rbDrive);
        ros::spinOnce();
        r.sleep();
    }
    delete steve;
}

void BoardControl::publishGPS(GPSData gps) {
    sensor_msgs::NavSatFix msg;
    msg.longitude = (((float)gps.longitude)/GPS_FLOAT_OFFSET)* -1.0;
    msg.latitude = (((float)gps.latitude)/GPS_FLOAT_OFFSET) ; // fix issue with -ve longitude
    msg.altitude = gps.altitude;
    
    if (gps.fixValid && gps.numSatelites >= MIN_SATELITES) {
        msg.status.status = msg.status.STATUS_FIX;
        ROS_DEBUG("Statelites %d", gps.numSatelites);
    } else {
        ROS_ERROR("Invalid fix, w/ only %d satelites", gps.numSatelites);
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

void BoardControl::publishBattery(double batteryVoltage) {
    std_msgs::Float64 msg;
    msg.data = batteryVoltage;
    battVoltPublisher.publish(msg);
}

void BoardControl::publishVoltmeter(double voltage) {
    std_msgs::Float64 msg;
    msg.data = voltage;
    voltmeterPublisher.publish(msg);
}

void BoardControl::publishIMU(IMUData imu) {
    geometry_msgs::Vector3 gyro_msg;
    geometry_msgs::Vector3 acc_msg;
    gyro_msg.x = imu.gx;
    gyro_msg.y = imu.gy;
    gyro_msg.z = imu.gz;
    acc_msg.x = imu.ax;
    acc_msg.y = imu.ay;
    acc_msg.z = imu.az;

    // Header just has dummy values
    gyroPublisher.publish(gyro_msg);
    accPublisher.publish(acc_msg);
}
//checks if the button state has changed and changes the feed
void BoardControl::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}


void BoardControl::controllerCallback(const sensor_msgs::Joy::ConstPtr& joy) {

    #define MID_IN 0
    #define DIFF 0.25
    
    float top = joy->axes[ARM_STICK_TOP] ;//* 0.2;
    float bottom = joy->axes[ARM_STICK_BOTTOM];//* 0.2;
    cameraBottomRotateIncRate = 0;
    cameraBottomTiltIncRate = 0;
    cameraTopRotateIncRate = 0;
    cameraTopTiltIncRate = 0;



	// Set sensitivity between 0 and 1:
    //  * 0 makes it output = input, 
    //  * 1 makes output = input ^3

    cameraBottomRotateIncRate = joy->axes[CAMERA_BOTTOM_ROTATE];
    cameraBottomTiltIncRate = joy->axes[CAMERA_BOTTOM_TILT];

    cameraTopRotateIncRate = joy->axes[CAMERA_TOP_ROTATE];
    cameraTopTiltIncRate = joy->axes[CAMERA_TOP_TILT];

    leftDrive = -joy->axes[LEFT_WHEELS];
    rightDrive = joy->axes[RIGHT_WHEELS];
    
    // Handle claw
    clawState = joy->axes[CLAW_STATE];
    rotState = joy->axes[CLAW_ROTATE];

    //Handle arm rotation
    armRotate = joy->axes[ARM_ROTATE];
    //armRotate = joy->axes[STICK_CH_LR];
    armIncRate = top * 5;
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
    	lDrive = power + (fabs(lr) * power);
    	rDrive = power;
    } else if (lr > 0){
    	lDrive = power;
    	rDrive = power - (fabs(lr) * power);
    } else {
    	lDrive = power;
    	rDrive = power;
    }
    
    lDrive = power;
    rDrive = power;
    leftDrive = lDrive;
    rightDrive = -rDrive;
    //leftDrive = (lDrive * 400) + MOTOR_MID;
    //rightDrive = (rDrive * 400) + MOTOR_MID;
    ROS_ERROR("%f,%f", leftDrive, rightDrive);

    // The formula in use i: output = (ax^3 + (1-a)x) * 500 + 1500
    // Where a = SENSITIVITY

    //leftDrive = (SENSITIVITY * pow(lDrive, 3) + (1 - SENSITIVITY) * lDrive);
    //rightDrive = (SENSITIVITY * pow(rDrive, 3) + (1 - SENSITIVITY) * rDrive);
}
