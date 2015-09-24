/*
 * PathingController is the node for reading in (subscribing to) position and destination information
 * and adjusting (publishing) twist velocities to the arduino controller
 * 
 * Author: Simon Ireland for Bluesat OWR
 * Date: 11/07/2015
 */
 
#ifndef PATH_CONTROL_H
#define PATH_CONTROL_H

#include <ros/ros.h>
#include "owr_messages/position.h"
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/NavSatFix.h>

class PathingController {
    
    public:
        PathingController(void);
        void spin();
    protected:
        void receivePosMsg(const owr_messages::position &msg) ;
        void receiveDestMsg(const boost::shared_ptr<sensor_msgs::NavSatFix const> & msg) ;
        void receiveDodgeMsg(const geometry_msgs::Twist::ConstPtr& velMsg);
        void sendMsg();
        ros::Publisher  twistPublisher;   
        ros::Subscriber positionSubscriber;
        ros::Subscriber destinationSubscriber;  
        ros::Subscriber  dodgeSubscriber;
    
    private:
        ros::NodeHandle node;
        std::string topic;
        
        float currPower;
        float currLR;
        geometry_msgs::Twist vel;

        double currLat;
        double currLong;
        double currHeading;
        double destHeading;
        double destLat;
        double destLong;
        double earthRadius;
        bool go;
};


#endif

