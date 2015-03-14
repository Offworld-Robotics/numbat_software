/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "ArduinoConverter.h"



int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    ArduinoConverter ArduinoConverter;

    ros::spin();
}

ArduinoConverter::ArduinoConverter() {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    
    //subscribe to joy stick
    //TODO: at some point we will need to handle two joysticks
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &ArduinoConverter::joyCallback, this);
    

        
}

//checks if the button state has changed and changes the feed
void ArduinoConverter::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}

void ArduinoConverter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    
    float power = joy->axes[DRIVE_AXES_UD];
    float lr = joy->axes[DRIVE_AXES_LR];
    
    float leftDrive  = 1.0f;
    float rightDrive = 1.0f;
    
    leftDrive  =   power - lr;
    rightDrive =   power + lr;
      
        
    fd = fopen(TTY, "w");
    if(fd) {
        fprintf(fd,"%f,%f\n",leftDrive,rightDrive);
        fclose(fd);
    } else {
        printf("unsucesfull\n");
    }
    printf("%f,%f\n",leftDrive,rightDrive);   
    //TODO: camera on/off
    //check if the camera button states have changes
    switchFeed(&cam0Button,joy->buttons[CAM_FEED_0],0);
    //TODO: camera rotation
    //TODO: take photo
    //TODO: map zoom in/out
}


