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
    n.setParam("groundStation","192.168.1.21");
    ros::Publisher ping_pub = n.advertise<std_msgs::Bool>("owr/ping",1000);
    while (ros::ok()){
        std_msgs::Bool networkStatus;
        std::string groundStation;
        n.getParam("groundStation",groundStation);
        //ping the ground station 1 times waiting for 0.2s between each packet with a timeout of 0.5 seconds
        std::string pingCommand = "ping -c 1 -w 0.5 -i 0.2 -W 1 ";
        pingCommand = pingCommand + groundStation;
        //stores the exit status of the system command that pings the given ip address
        int exitStatus = system(pingCommand.c_str());
        //if the exit status is 0, the ping succeeded and thus a connection to the ip address (ground station) exists
        //publish that there is a network connection
        if (exitStatus==0) {
            networkStatus.data = true;
        } else {
            ROS_INFO("Publish dropped out");
            networkStatus.data = false; //otherwise publish that the network connection has dropped out
        }
        ping_pub.publish(networkStatus);
        ros::spinOnce();
    }
    return EXIT_SUCCESS;   
}
