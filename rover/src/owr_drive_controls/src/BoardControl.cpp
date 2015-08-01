/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "BoardControl.h"
 #include "Bluetongue.h"
 #include <assert.h>
 
 #define MOTOR_MID 1500

static void printStatus(struct status *s) {
	cout << "Battery voltage: " << s->batteryVoltage << endl;
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
    //subscribe to xbox controller
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &BoardControl::joyCallback, this);
    armSubscriber = nh.subscribe<sensor_msgs::Joy>("arm_joy", 10, &BoardControl::armCallback, this);
    leftDrive = MOTOR_MID;
    rightDrive = MOTOR_MID; 
    armTop = MOTOR_MID;
    armBottom = MOTOR_MID;
    armRotate = 0.5;
          
}

void BoardControl::run() {
    Bluetongue* steve = new Bluetongue(TTY);

    while(ros::ok()) {
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
    #define MAX_IN 1.0
    #define DIFF 0.25

	// Set sensitivity between 0 and 1, 0 makes it output = input, 1 makes output = input ^3
    #define SENSITIVITY 1.0
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
    #define MAX_IN 1.5
    #define DIFF 0.25
    
    float top = joy->axes[STICK_R_UD] ;//* 0.2;
    float bottom = (joy->axes[STICK_L_UD]) ;//* 0.2;
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    armRotate = joy->axes[STICK_CH_LR];
 
    armTop = (top / MAX_IN) * 500 + 1500  ;
    armBottom = (bottom / MAX_IN) * 500 + 1500  ;
}
