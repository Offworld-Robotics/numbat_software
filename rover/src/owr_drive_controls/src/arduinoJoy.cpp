/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "ArduinoConverter.h"
 #include <assert.h>



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
    
    float power = joy->axes[DRIVE_AXES_UD] * 0.2;
    float lr = (-joy->axes[STICK_L_LR]) * 0.2;
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    
    float lDrive  =   power + lr;
    float rDrive =   -(power - lr);
    
    leftDrive = ((lDrive / MAX_IN) * 500) + 1500.0  ;
    rightDrive = ((rDrive / MAX_IN) * 500) + 1500.0  ;
    
    /*if (joy->axes[STICK_LT]) {
        lfDrive = leftDrive;
        lmDrive = leftDrive;
        lbDrive = leftDrive;
        rfDrive = rightDrive * DIFF;
        rmDrive = rightDrive * DIFF;
        rbDrive = rightDrive * DIFF;
    } else if (joy->axes[STICK_RT]) {
        lfDrive = leftDrive * DIFF;
        lmDrive = leftDrive * DIFF;
        lbDrive = leftDrive * DIFF;
        rfDrive = rightDrive;
        rmDrive = rightDrive;
        rbDrive = rightDrive;
    } else {*/
        lfDrive = leftDrive;
        lmDrive = leftDrive;
        lbDrive = leftDrive;
        rfDrive = rightDrive;
        rmDrive = rightDrive;
        rbDrive = rightDrive;
    //}
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


