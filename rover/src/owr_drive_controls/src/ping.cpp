/*
 * Pings the ground station to test the network connection
 * Author: Elliott Smith for BlueSat OWR
 * Date 3/4/2016
 */



#include <assert.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <limits>


int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_ping_node");
    ros::NodeHandle n;
    n.setParam("groundStation","192.168.1.3");
    ros::Publisher ping_pub = n.advertise<std_msgs::Bool>("owr/ping",10);
    while (ros::ok()){
        std_msgs::Bool networkStatus;
        char* groundStation;
        n.getParam("groundStation",groundStation);
        ssh groundStation;
        ping groundStation;
        networkStatus.data = true; //TODO:for now just constantly claim the network status is good. 
        ping_pub.publish(networkStatus);
        ros::spinOnce();
    }
    return EXIT_SUCCESS;   
}