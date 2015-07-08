/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "BoardControl.h"
 #include "Bluetongue.h"
 #include <assert.h>

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
    leftDrive = 1500.0;
    rightDrive = 1500.0;       
}

void BoardControl::run() {
    Bluetongue* steve = new Bluetongue("/dev/ttyACM0");

    while(ros::ok()) {
        struct status s = steve->update((leftDrive-1500.0)/1000, (rightDrive-1500.0)/1000);
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
    #define SENSITIVITY 1

    float power = joy->axes[DRIVE_AXES_UD];
    float lr = (-joy->axes[STICK_L_LR]);
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    
    float lDrive  =   (power + lr)/2;
    float rDrive =   -(power - lr)/2;
    
    // The formula in use i: output = (ax^3 + (1-a)x) * 500 + 1500
    // Where a = SENSITIVITY

    leftDrive = ((SENSITIVITY * pow(lDrive, 3) + (1 - SENSITIVITY) * lDrive) * 500) + 1500.0;
    rightDrive = ((SENSITIVITY * pow(rDrive, 3) + (1 - SENSITIVITY) * rDrive) * 500) + 1500.0;
    
}


