/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 7/02/15
 */
 
 #include "ArduinoMagControl.h"
 #include <assert.h>
 #include <ros/ros.h>


int main(int argc, char ** argv) {
    ros::init(argc, argv, "owr_arduino_mag");
    ArduinoMagControl ArduinoMagControl;
    ArduinoMagControl.run();
    
}

ArduinoMagControl::ArduinoMagControl() {

    fd = fopen(TTY, "r");
    assert(fd != NULL);
    //publish headings, with latch on
    pub = node.advertise<owr_messages::heading>("/owr/heading",1,true);
        
}

void ArduinoMagControl::run() {
    #define MAG_BUFFER 20
    ros::Rate rate(24.);
    while(ros::ok()) {    
        if(fd) {
            char buffer[MAG_BUFFER];
            fgets(buffer,MAG_BUFFER,fd);
            sscanf(buffer, "%lf", &heading);
            ROS_INFO("%lf", heading);
        } else {
            ROS_ERROR("unsuccesful write");
            
        }  
        ros::spinOnce();
        rate.sleep();
    }
}

