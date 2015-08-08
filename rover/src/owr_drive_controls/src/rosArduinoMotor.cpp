/*
 * Converts Twist message input (subscribe) to arduino output, etc
 * Author: Simon Ireland for BlueSat OWR
 * Start: 20/06/15
 */

 #include "RosArduinoMotor.h"
 #include <assert.h>



int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_telop");
    RosArduinoMotor rosArduinoMotor;
    rosArduinoMotor.run();
}

RosArduinoMotor::RosArduinoMotor() {

    // Init file
    fd = fopen(TTY, "w");
    assert(fd != NULL);

    //A subscriber for Twist messages
    //uses api at https://github.com/bluesat/owr_software/wiki/OWR-ROS-API
    velSubscriber = nh.subscribe<geometry_msgs::Twist>("owr/control/drive", 10, &RosArduinoMotor::velCallback, this);

    // Set natural state for motors
    leftDrive = 1500.0;
    rightDrive = 1500.0;
    lfDrive = leftDrive;
    lmDrive = leftDrive;
    lbDrive = leftDrive;
    rfDrive = rightDrive;
    rmDrive = rightDrive;
    rbDrive = rightDrive;

}

void RosArduinoMotor::run() {
    while(ros::ok()) {
    	sendMessage(lfDrive,lmDrive,lbDrive,rfDrive,rmDrive,rbDrive);
        ros::spinOnce();
    }
}

// Send a message to the arduino so it can run the motors
void RosArduinoMotor::sendMessage(float lf, float lm, float lb, float rf, float rm, float rb) {
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

// Convert subscribed Twist input to motor vectors for arduino output
void RosArduinoMotor::velCallback(const geometry_msgs::Twist::ConstPtr& vel) {

	#define MAX_IN 1.0
    #define DIFF 0.25

	// Set sensitivity between 0 and 1, 0 makes it output = input, 1 makes output = input ^3
    #define SENSITIVITY 1

    float power = vel->linear.x;
    float lr = vel->angular.x;

    //float leftDrive  = 1.0f;
    //float rightDrive = 1.0f;

    float lDrive  =   (power + lr)/2;
    float rDrive =   -(power - lr)/2;

    // The formula in use i: output = (ax^3 + (1-a)x) * 500 + 1500
    // Where a = SENSITIVITY

    leftDrive = ((SENSITIVITY * pow(lDrive, 3) + (1 - SENSITIVITY) * lDrive) * 500) + 1500.0;
    rightDrive = ((SENSITIVITY * pow(rDrive, 3) + (1 - SENSITIVITY) * rDrive) * 500) + 1500.0;

    lfDrive = leftDrive;
    lmDrive = leftDrive;
    lbDrive = leftDrive;
    rfDrive = rightDrive;
    rmDrive = rightDrive;
    rbDrive = rightDrive;
}
