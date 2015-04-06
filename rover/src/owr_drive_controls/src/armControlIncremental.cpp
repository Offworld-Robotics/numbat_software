/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "ArmControlIncremental.h"
 #include <assert.h>



int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    ArmConverter armConverter;
    armConverter.run();
    
}

ArmConverter::ArmConverter() {

    //init button sates
    cam0Button = 0;
    cam1Button = 0;
    cam2Button = 0;
    cam3Button = 0;
    fd = fopen(TTY, "w");
    assert(fd != NULL);
    //subscribe to joy stick
    //TODO: at some point we will need to handle two joysticks
    joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 1, &ArmConverter::joyCallback, this);
    topDrive = 1500.0;
    bottomDrive = 1500.0;
        
}

void ArmConverter::run() {
    ros::Rate rate(24.);
    while(ros::ok()) {
        sendMessage(topDrive,bottomDrive);
        
        ros::spinOnce();
        rate.sleep();
    }
}

void ArmConverter::sendMessage(int tm, int bm) {
    if(fd) {
        fprintf(fd,"%d %d\n %d %d\n",TOP_ACTUATOR,tm,BOTTOM_ACTUATOR,bm);
        float buffer;
        //fscanf(fd, "%f", &buffer);
        //printf("%f", buffer);
        //fsync((int)fd);
        //fflush(fd);
    } else {
        printf("unsucesfull\n");
    }    
    printf("%d %d\n %d %d\n",TOP_ACTUATOR,tm,BOTTOM_ACTUATOR,bm);   
}

//checks if the button state has changed and changes the feed
void ArmConverter::switchFeed(int * storedState, int joyState, int feedNum) {
    if((*storedState) != joyState) {
        //TODO: switch feed
    } 
}

void ArmConverter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    #define MAX_IN 1.5
    #define DIFF 0.25
    
    float top = joy->axes[STICK_R_UD] ;//* 0.2;
    float bottom = (-joy->axes[STICK_L_UD]) ;//* 0.2;
    
    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;
    
    float topD;
    if ( top > 0) {
        topD = topDrive + INCREMENT;
    } else if (top < 0) {
        topD = topDrive - INCREMENT;
    }
    
    float bottomD;
    if ( bottom > 0) {
        bottomD = bottomDrive + INCREMENT;
    } else if (top < 0) {
        bottomD = bottomDrive - INCREMENT;
    }
    if(!(topD < FULL_EXTENSION && topD > FULL_RETRACTION)) {
        topD = topDrive;
    } 
    if(!(bottomD < FULL_EXTENSION && bottomD > FULL_RETRACTION)) {
        bottomD = bottomDrive;
    }
    
    sendMessage(topDrive,bottomDrive);    
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


