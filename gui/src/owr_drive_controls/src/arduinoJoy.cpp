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

        
}

void ArduinoConverter::run() {
    while(ros::ok()) {
        if(fd) {
            fprintf(fd,"%f %f\n",leftDrive,rightDrive);
            float buffer;
            //fscanf(fd, "%f", &buffer);
            //printf("%f", buffer);
            //fsync((int)fd);
            //fflush(fd);
        } else {
            printf("unsucesfull\n");
        }    
        ros::spinOnce();
    }
}

//checks if the button state has changed and changes the feed
void ArduinoConverter::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}

void ArduinoConverter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    #define MAX_IN 0.5
    
    float power = joy->axes[DRIVE_AXES_UD] * 0.3;
    float lr = joy->axes[STICK_L_LR] * 0.1;
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    
    leftDrive  =   power + lr;
    rightDrive =   -(power - lr);
    
    leftDrive = ((leftDrive / MAX_IN) * 500) + 1500.0  ;
    rightDrive = ((rightDrive / MAX_IN) * 500) + 1500.0  ;
        
    /*if(!fd) {
        fd = fopen(TTY, "w");
        printf("reopen\n");
    }*/
    if(fd) {
        fprintf(fd,"%f %f\n",leftDrive,rightDrive);
        float buffer;
        //fscanf(fd, "%f", &buffer);
        //printf("%f", buffer);
        //fsync((int)fd);
        //fflush(fd);
    } else {
        printf("unsucesfull\n");
    }
    

    printf("%f %f\n",leftDrive,rightDrive);   
   
    //TODO: camera on/off
    //check if the camera button states have changes
    switchFeed(&cam0Button,joy->buttons[CAM_FEED_0],0);
    //TODO: camera rotation
    //TODO: take photo
    //TODO: map zoom in/out
}


