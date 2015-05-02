/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "ArduinoConverter.h"
 #include <assert.h>

#define CENTRE 1500
#define MAX_POWER 2000
#define MIN_POWER 1000

/*
 * The vecotr multipliers for individual wheels
 * MAKE SURE -1 <= value <= 1   !!!!!
 * where -1 means it is going backwards, so if you are making it turn
 * make sure one side has negative values
 */

#define VECTOR_LF 1
#define VECTOR_LM 1
#define VECTOR_LB 1
#define VECTOR_RF 1
#define VECTOR_RM 1
#define VECTOR_RB 1


int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    ArduinoConverter arduinoConverter;
    arduinoConverter.run();
    
}

ArduinoConverter::ArduinoConverter() {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    fd = fopen(TTY, "w");
    assert(fd != NULL);
    //subscribe to joy stick
    //TODO: at some point we will need to handle two joysticks
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 1, &ArduinoConverter::joyCallback, this);
    leftDrive = 1500.0;
    rightDrive = 1500.0;
    lfDrive = leftDrive;
    lmDrive = leftDrive;
    lbDrive = leftDrive;
    rfDrive = rightDrive;
    rmDrive = rightDrive;
    rbDrive = rightDrive;
        
}

void ArduinoConverter::run() {
    while(ros::ok()) {
        sendMessage(lfDrive,lmDrive,lbDrive,rfDrive,rmDrive,rbDrive);
        ros::spinOnce();
    }
}

void ArduinoConverter::sendMessage(float lf, float lm, float lb, float rf, float rm, float rb) {
    if(fd) {
        fprintf(fd,"%f %f %f %f %f %f\n",lf,lm,lb,rf,rm,rb);
        float buffer;
        //fscanf(fd, "%f", &buffer);
        //printf("%f", buffer);
        //fsync((int)fd);
        //fflush(fd);
    } else {
        printf("unsucesfull\n");
    }    
    printf("%f %f %f %f %f %f\n",leftDrive,leftDrive,leftDrive,rightDrive,rightDrive,rightDrive);   
}

//checks if the button state has changed and changes the feed
void ArduinoConverter::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}

void ArduinoConverter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    #define MAX_IN 0.5
    #define DIFF 0.25
    
    if(joy->buttons[BUTTON_A]) {
        
        float power = joy->axes[DRIVE_AXES_UD] * 0.2;
        float lr = (-joy->axes[STICK_L_LR]) * 0.2;
        
        //float leftDrive  = 1.0f;
        //float rightDrive = 1.0f;
        
        float lDrive  =   power + lr;
        float rDrive =   -(power - lr);
        
        leftDrive = ((lDrive / MAX_IN) * 500) + 1500.0  ;
        rightDrive = ((rDrive / MAX_IN) * 500) + 1500.0  ;
    
    }
    
    /* Checks whether a shoulder button is being pressed
     * if so, make the rover turn in the direction of the held
     * button (ie. LB = turn left)
     * Otherwise, go straight
     * Simon Ireland, 25/04/15
     *
     * Added vector testers for checking individual speeds of motors
     */
	//if (joy->buttons[LB]) {
		lfDrive = leftDrive * VECTOR_LF;
		lmDrive = leftDrive * VECTOR_LM;
		lbDrive = leftDrive * VECTOR_LB;
		rfDrive = rightDrive * VECTOR_RF;
		rmDrive = rightDrive * VECTOR_RM;
		rbDrive = rightDrive * VECTOR_RB;
	//} else if (joy->buttons[RB]) {
	/*
		lfDrive = leftDrive;
		lmDrive = leftDrive;
		lbDrive = leftDrive;
		rfDrive = rightDrive * DIFF;
		rmDrive = rightDrive * DIFF;
		rbDrive = rightDrive * DIFF;
	} else {
		lfDrive = leftDrive;
		lmDrive = leftDrive;
		lbDrive = leftDrive;
		rfDrive = rightDrive;
		rmDrive = rightDrive;
		rbDrive = rightDrive;
    } */
    /*if(!fd) {
        fd = fopen(TTY, "w");
        printf("reopen\n");
    }*/
    /*if(fd) {
        fprintf(fd,"%f %f %f %f %f %f\n",leftDrive,leftDrive,leftDrive,rightDrive,rightDrive,rightDrive);
        float buffer;
        //fscanf(fd, "%f", &buffer);
        //printf("%f", buffer);
        //fsync((int)fd);
        //fflush(fd);
    } else {
        printf("unsucesfull\n");
    }*/
    

   
   
    //TODO: camera on/off
    //check if the camera button states have changes
    switchFeed(&cam0Button,joy->buttons[CAM_FEED_0],0);
    //TODO: camera rotation
    //TODO: take photo
    //TODO: map zoom in/out
}


