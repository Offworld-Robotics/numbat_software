/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
#include "BoardControl.h"
#include "Bluetongue.h"
#include "RoverDefs.h"
#include "ButtonDefs.h"
#include "Bluetounge2Params.h"

#include <assert.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include "owr_messages/adc.h"
#include <limits>
#include <math.h>

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

#define SPINNER_THREADS 4

// Set sensitivity between 0 and 1, 0 makes it output = input, 1 makes output = input ^3
#define SENSITIVITY 1

static inline void printStatus(struct status *s) {
    ROS_INFO("Battery voltage: %f", s->batteryVoltage);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    BoardControl BoardControl;
    BoardControl.run();
}

BoardControl::BoardControl() : 
    nh(),
    jMonitor(nh),
    frontLeftWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        WHEEL_GEARS,
        WHEEL_N_GEARS,
        "/front_left_wheel_axel_controller/command",
        nh,
        "front_left_wheel_axel"
    ),
    frontRightWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        WHEEL_GEARS,
        WHEEL_N_GEARS,
        "/front_right_wheel_axel_controller/command",
        nh,
        "front_right_wheel_axel"
    ), backLeftWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        WHEEL_GEARS,
        WHEEL_N_GEARS,
        "/back_left_wheel_axel_controller/command",
        nh,
        "back_left_wheel_axel"
    ),
    backRightWheel(
        WHEEL_MOTOR_MIN_PWM,
        WHEEL_MOTOR_MAX_PWM,
        WHEEL_MOTOR_RPM,
        WHEEL_RADIUS,
        WHEEL_GEARS,
        WHEEL_N_GEARS,
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
        SWERVE_ACCURACY,
        "/front_left_swerve_controller/command",
        nh,
        "front_left_swerve"
    ),
    frontRightSwerve(
       SWERVE_RADIUS,
        SWERVE_GEARS,
        SWERVE_N_GEARS,
        SWERVE_MOTOR_MIN_PWM,
        SWERVE_MOTOR_MAX_PWM,
        SWERVE_MOTOR_RPM,
        SWERVE_ACCURACY,
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
        SWERVE_ACCURACY,
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
        SWERVE_ACCURACY,
        "/back_right_swerve_controller/command",
        nh,
        "back_right_swerve"
    ),
    armBaseRotate(
        ARM_BASE_ROTATE_RADIUS,
        ARM_BASE_ROTATE_GEARS,
        ARM_BASE_ROTATE_N_GEARS,
        ARM_BASE_ROTATE_MOTOR_MIN_PWM,
        ARM_BASE_ROTATE_MOTOR_MAX_PWM,
        ARM_BASE_ROTATE_MOTOR_RPM,
        ARM_ACCURACY,
        "/arm_base_rotate_controller/command",
        nh,
        "arm_base_rotation"
    ),
    armUpperAct(
        UPPER_MIN_ADC,
        UPPER_MAX_ADC,
        UPPER_MIN_POS,
        UPPER_MAX_POS,
        ARM_ACT_PWM_MIN, 
        ARM_ACT_PWM_MAX, 
        "/upper_arm_act_controller/command", 
        nh, 
        "upper_arm_act"
    ),
    armLowerAct(
        LOWER_MIN_ADC,
        LOWER_MAX_ADC,
        LOWER_MIN_POS,
        LOWER_MAX_POS,
        ARM_ACT_PWM_MIN, 
        ARM_ACT_PWM_MAX, 
        "/lower_arm_act_controller/command", 
        nh, 
        "lower_arm_act"
    ),
    lidar(
        STATIONARY,
        "/laser_tilt_joint_controller/command",
        nh,
        "laser_tilt_joint"
    ),
    frontLeftSwervePotMonitor(SWERVE_POT_L_LIMIT_N_DEG, SWERVE_POT_L_LIMIT_P_DEG, SWERVE_POT_L_REVOLUTION, SWERVE_POT_TURNS, SWERVE_POT_L_CENTER),
    frontRightSwervePotMonitor(SWERVE_POT_R_LIMIT_N_DEG, SWERVE_POT_R_LIMIT_P_DEG, SWERVE_POT_R_REVOLUTION, SWERVE_POT_TURNS, SWERVE_POT_R_CENTER),
    backLeftSwervePotMonitor(SWERVE_POT_L_LIMIT_N_DEG, SWERVE_POT_L_LIMIT_P_DEG, SWERVE_POT_L_REVOLUTION, SWERVE_POT_TURNS, SWERVE_POT_L_CENTER),
    backRightSwervePotMonitor(SWERVE_POT_R_LIMIT_N_DEG, SWERVE_POT_R_LIMIT_P_DEG, SWERVE_POT_R_REVOLUTION, SWERVE_POT_TURNS, SWERVE_POT_R_CENTER),
    armRotationBasePotMonitor(ARM_POT_LIMIT_N_DEG, ARM_POT_LIMIT_P_DEG, ARM_POT_REVOLUTION, ARM_POT_TURNS, ARM_POT_CENTER),
    asyncSpinner(SPINNER_THREADS),
    clawRotateTrim(0)
    {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    ros::TransportHints transportHints = ros::TransportHints().tcpNoDelay();
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("/owr/joysticks",2, &BoardControl::controllerCallback, this, transportHints);
    rotateTrimSub = nh.subscribe<std_msgs::Int32>("/owr/claw_rotate_trim",1, &BoardControl::trimCallback, this, transportHints);
    gpsPublisher = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix",  10);
    battVoltPublisher = nh.advertise<std_msgs::Float64>("battery_voltage", 10);
    voltmeterPublisher = nh.advertise<std_msgs::Float64>("voltmeter", 10);
    velSubscriber = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &BoardControl::velCallback, this, transportHints);
    adcStatusPublisher = nh.advertise<owr_messages::adc>("/owr/adc", 10, true);
    leftDrive = MOTOR_MID;
    rightDrive = MOTOR_MID; 
    armTop = MOTOR_MID;
    armBottom = MOTOR_MID;
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
    
    adcMsgSeq = 0;
    
    currentVel.linear.x = 0;
    currentVel.linear.y = 0;
    currentVel.linear.z = 0;
    
    //swerve setup
    jMonitor.addJoint(&frontLeftWheel);
    jMonitor.addJoint(&frontRightWheel);
    jMonitor.addJoint(&backLeftWheel);
    jMonitor.addJoint(&backRightWheel);
    jMonitor.addJoint(&frontLeftSwerve);
    jMonitor.addJoint(&frontRightSwerve);
    jMonitor.addJoint(&lidar);
    jMonitor.addJoint(&armBaseRotate);
    jMonitor.addJoint(&armLowerAct);
    jMonitor.addJoint(&armUpperAct);
    
    
    armRotateAngle = 0.0;
    
    //velocity setup
    currentVel.linear.x = std::numeric_limits<double >::quiet_NaN();
}

int clawRotScale(int raw) {
    return ((float)raw/(float)CLAW_ROTATION_MAX)*1000.0 + 1000;
}

int reverseClawRotScale(int actual) {
    return (actual - 1000)*CLAW_ROTATION_MAX/1000.0; 
}

int cameraRotScale(int raw) {
    return ((float)raw/(float)CAMERA_ROTATION_MAX)*1000.0 + 1000;
}


void cap(int *a, int low, int high) {
    *a = *a < low ? low : *a;
    *a = *a > high ? high : *a;
}

void capf(float *a, float low, float high) {
    *a = *a < low ? low : *a;
    *a = *a > high ? high : *a;
}


#define UPDATE_RATE 10 //Hz
#define SECONDS_2_NS 1000000000
#define UPDATE_RATE_NS ((1/UPDATE_RATE)*SECONDS_2_NS)
#define ESTIMATE_INTERVAL_NS ((1/30)*SECONDS_2_NS) //30Hz
#define N_UPDATES 3

void BoardControl::run() {
    std::string board;
    bool firstRun = true;
    nh.param<std::string>("board_tty", board, TTY);
    ROS_INFO("connecting to board on %s", board.c_str());
    Bluetongue* steve = new Bluetongue(board.c_str());
    struct status s;
    s.isConnected = true;
    ros::Rate r(UPDATE_RATE);
    int cbr = 0, cbt = 0;
    int pwmFLW, pwmFRW, pwmBLW, pwmBRW, pwmFLS, pwmFRS, pwmBLS, pwmBRS;
    int pwmArmTop, pwmArmBottom, pwmArmRot;
    int pwmClawRotate;//, pwmClawGrip;
    int pwmLIDAR;
    int pwmCamBTilt, pwmCamBRot, pwmCamTTilt, pwmCamTRot;
    
    //make sure we don't send the rover crazy when we start
    //this prevents garbage be thrown at the rover :P
    pwmFLW = MOTOR_MID;
    pwmFRW = MOTOR_MID;
    pwmBLW = MOTOR_MID;
    pwmBRW = MOTOR_MID;
    pwmFLS = MOTOR_MID;
    pwmFRS = MOTOR_MID;
    pwmBLS = MOTOR_MID;
    pwmBRS = MOTOR_MID;

    pwmArmTop = MOTOR_MID;
    pwmArmBottom = MOTOR_MID;
    pwmArmRot = MOTOR_MID;

    pwmClawRotate = MOTOR_MID;
    pwmClawGrip = MOTOR_MID;

    pwmLIDAR = MOTOR_MID;

    pwmCamBTilt = MOTOR_MID;
    pwmCamBRot = MOTOR_MID;
    pwmCamTTilt = MOTOR_MID;
    pwmCamTRot = MOTOR_MID;
    asyncSpinner.start();
    ros::Time lastUpdate = ros::Time::now();
    //initialise but don't publish untill we have data
    while (ros::ok()) {
        //so the first time dosen't fail
        jMonitor.beginCycle(ros::Time::now(), UPDATE_RATE_NS, ESTIMATE_INTERVAL_NS,0); 
        while(ros::ok()) {
            
            //cbr = cbr < 120 ? cbr + 5 : 0;
            //cbt = cbt < 70 ? cbt + 5 : 0;
            //armTop += armIncRate;
            cap(&armTop, MOTOR_MIN, MOTOR_MAX);
            
            cameraBottomRotate += cameraBottomRotateIncRate;
            cap(&cameraBottomRotate, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);
            
            cameraBottomTilt += cameraBottomTiltIncRate;
            cap(&cameraBottomTilt, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);
            
            cameraTopRotate += cameraTopRotateIncRate;
            cap(&cameraTopRotate, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);
            
            cameraTopTilt += cameraTopTiltIncRate;
            cap(&cameraTopTilt, CAMERA_ROTATION_MIN, CAMERA_ROTATION_MAX);

            if (clawState  == OPEN) {
                clawGrip += 1;
            } else if (clawState  == CLOSE) {
                clawGrip -=  1;
            }
            //cap(&clawGrip, CLAW_ROTATION_MIN, CLAW_ROTATION_MAX); 
            
            //cameraBottomTilt = cbt;
            //cameraBottomRotate = cbr;
            jMonitor.endCycle(ros::Time::now());
            s = steve->update(
                pwmFLW, pwmFRW, pwmBLW, pwmBRW, pwmFLS, pwmFRS,
                pwmArmTop, pwmArmBottom,pwmArmRot,
                pwmClawRotate+clawRotateTrim, pwmClawGrip,
                pwmCamBRot, pwmCamBTilt, pwmCamTRot, pwmCamTTilt,
                pwmLIDAR
            );
            if (!s.isConnected) break;
            double updateRateNSec = (ros::Time::now() - lastUpdate).toNSec();
            double updateRateHZ = 1.0/( updateRateNSec / SECONDS_2_NS);
            ROS_INFO("Update Rate NSec: %f, HZ: %f", updateRateNSec, updateRateHZ);
            lastUpdate = ros::Time::now();
            publishADC(s); 
            jMonitor.beginCycle(lastUpdate, updateRateNSec, ESTIMATE_INTERVAL_NS, N_UPDATES);
            /*armRotationBasePotMonitor.updatePos(s.enc0, lastUpdate);*/
            //TODO: when using encoders s.enc0 was fliped, check this is not the case for pot
            frontLeftSwervePotMonitor.updatePos(s.swerveLeft, lastUpdate);
            frontRightSwervePotMonitor.updatePos(s.swerveRight, lastUpdate);
            armRotationBasePotMonitor.updatePos(s.pot0, lastUpdate);
            
            if(firstRun) {
	        armRotateAngle = armRotationBasePotMonitor.getPosition();
                clawGrip = reverseClawRotScale(s.clawActual);
                firstRun = false;
            }
            //do joint calculations
            motorVels swerveState = swerveDrive.doVelTranslation(&(currentVel));
            pwmFLW = frontLeftWheel.velToPWM(swerveState.frontLeftMotorV);
            pwmFRW = frontRightWheel.velToPWM(swerveState.frontRightMotorV);
            pwmBLW = backLeftWheel.velToPWM(swerveState.backLeftMotorV);
            pwmBRW = backRightWheel.velToPWM(swerveState.backRightMotorV);
            //TODO: this should actually be the angle from the encoders
            pwmFRS = frontRightSwerve.posToPWM(frontRightSwervePotMonitor.getPosition(), updateRateHZ);
            pwmFLS =  frontLeftSwerve.posToPWM(-frontLeftSwervePotMonitor.getPosition(), updateRateHZ);
            pwmLIDAR = lidar.velToPWM(updateRateHZ);
            
            //adjust the arm position
            armRotateAngle += armRotateRate;
            armRotateAngle = fmax(armRotationBasePotMonitor.getMinAngle(),fmin(armRotationBasePotMonitor.getMaxAngle(), armRotateAngle));
            //pwmArmRot = (armRotateRate * 500) + 1500;
            pwmArmRot = armBaseRotate.posToPWM(armRotateAngle,armRotationBasePotMonitor.getPosition(), updateRateHZ); 
            ROS_INFO("Arm Rotate %d", pwmArmRot);
            
            //for now do this for actuators
            //pwmArmTop = armTop;
            pwmArmTop = armUpperAct.velToPWM(armTop);
            armUpperAct.updatePos(s.armUpper);
            
            //pwmArmBottom = armBottom;
            pwmArmBottom = armLowerAct.velToPWM(armBottom);
            armLowerAct.updatePos(s.armLower);
            
            //and keep everything else the same
            //pwmClawRotate = clawRotScale(clawRotate);
            if (rotState == ANTICLOCKWISE) {
                pwmClawRotate = 1510;
            } else if (rotState == CLOCKWISE) {
                pwmClawRotate = 1490;
            } else { //STOP
            	pwmClawRotate = 1500;
            }
            pwmClawGrip   = clawRotScale(clawGrip);
            pwmCamBRot    = cameraRotScale(cameraBottomRotate);
            pwmCamBTilt   = cameraRotScale(cameraBottomTilt);
            pwmCamTRot    = cameraRotScale(cameraTopRotate);
            pwmCamTTilt   = cameraRotScale(cameraTopTilt);

            //publish sensors data
            publishGPS(s.gpsData);
            publishBattery(s.batteryVoltage);
            #ifdef VOLTMETER_ON
            publishVoltmeter(s.voltmeter);
            #endif
        }
        ROS_ERROR("Lost usb connection to Bluetongue");
        ROS_ERROR("Trying to reconnect every %d ms", RECONNECT_DELAY);
        bool success = false;
        firstRun = true;
        while (ros::ok() && !success) {
            usleep(RECONNECT_DELAY * 1000);
            success = steve->reconnect();
            if (success) ROS_INFO("Bluetongue reconnected!");
            else ROS_WARN("Bluetongue reconnection failed. Trying again soon...");
        }
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

void BoardControl::publishADC(status s) {
   owr_messages::adc adcMsg;
   adcMsg.pot.push_back(s.swerveLeft);
   adcMsg.potFrame.push_back("front_left_swerve");
   adcMsg.pot.push_back(s.swerveRight);
   adcMsg.potFrame.push_back("front_right_swerve");
   adcMsg.pot.push_back(s.pot0);
   adcMsg.potFrame.push_back("pot0");
   adcMsg.pot.push_back(s.pot1);
   adcMsg.potFrame.push_back("pot1");
   adcMsg.pot.push_back(s.pot2);
   adcMsg.potFrame.push_back("pot2");
   adcMsg.pot.push_back(s.pot3);
   adcMsg.potFrame.push_back("pot3");
   adcMsg.pot.push_back(s.armLower);
   adcMsg.potFrame.push_back("armLower");
   adcMsg.pot.push_back(s.armUpper);
   adcMsg.potFrame.push_back("armUpper");
   adcMsg.pot.push_back(s.clawActual);
   adcMsg.potFrame.push_back("clawActual");
   adcMsg.pot.push_back(s.clawEffort);
   adcMsg.potFrame.push_back("clawEffort");
   adcMsg.pot.push_back(pwmClawGrip);
   adcMsg.potFrame.push_back("clawGrip");
   adcMsg.header.stamp = ros::Time::now();
   adcMsg.header.seq = (++adcMsgSeq);
   adcStatusPublisher.publish<owr_messages::adc>(adcMsg);
}
    

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
//     cameraBottomRotateIncRate = 0;
//     cameraBottomTiltIncRate = 0;
//     cameraTopRotateIncRate = 0;
//     cameraTopTiltIncRate = 0;



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
    //armRotateRate = joy->axes[ARM_ROTATE] * ARM_INCE_RATE_MULTIPLIER;
    armRotateRate = (joy->axes[ARM_ROTATE]*ARM_ROTATE_RATE);
    //armIncRate = top * 5;
    armBottom = (bottom / MAX_IN) * 500 + MOTOR_MID  ;
    armTop = (top / MAX_IN) * 500 + MOTOR_MID  ;
    
    if(joy->buttons[FL_SWERVE_RESET]) {
        frontLeftSwervePotMonitor.resetPos();
        ROS_INFO("Reset Front left Swerve");
    }
    if (joy->buttons[FR_SWERVE_RESET]) {
        frontRightSwervePotMonitor.resetPos();
        ROS_INFO("Reset Front right Swerve");
    }
}


void BoardControl::velCallback(const nav_msgs::Odometry::ConstPtr& vel) {
    currentVel = (vel->twist.twist);
}



void BoardControl::trimCallback(const std_msgs::Int32::ConstPtr& trimMsg) {
    clawRotateTrim = trimMsg->data;
}
