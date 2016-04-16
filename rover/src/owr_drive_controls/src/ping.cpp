/*
 * Pings the ground station to test the network connection
 * Author: Elliott Smith for BlueSat OWR
 * Date 3/4/2016
 */



#include <assert.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <limits>
#include <string.h>


int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_ping_node");
    ros::NodeHandle n;
    n.setParam("groundStation","127.0.0.1");
    ros::Publisher ping_pub = n.advertise<std_msgs::Bool>("owr/ping",10);
    while (ros::ok()){
        std_msgs::Bool networkStatus;
        std::string groundStation;
        n.getParam("groundStation",groundStation);
        //ping the ground station 3 times waiting for 0.5 seconds each time
        std::string pingCommand = "ping -c 3 -w 0.5 ";
        pingCommand = pingCommand + groundStation;
        int exitStatus = system(pingCommand.c_str());
        if (exitStatus==0) {
            networkStatus.data = true;
        } else {
            networkStatus.data = false;
        }
        ping_pub.publish(networkStatus);
        ros::spinOnce();
    }
    return EXIT_SUCCESS;   
}