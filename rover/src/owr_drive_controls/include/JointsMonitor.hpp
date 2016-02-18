/*
 * Handles the monitoring of joints for transforms
 * Original Author: Harry J.E Day
 * NOTE: Joint Publisher code based on code originally by Simon in Bluetounge.cpp
 * Editors:
 * Date Started: 17/02/2016
 * ros_package: owr_drive_controls
 * ros_node: owr_board_control
 */
#ifndef JOINTS_MONITOR_H
#define JOINTS_MONITOR_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "JointController.hpp"


class JointsMonitor {
    public:
        JointsMonitor(ros::NodeHandle nodeHandle);
        
        /**
         * functions to triger the start  of the handler
         * Params:
         *      startTime - the time this update cycle began at
         *      updateFrequencyNsecs - the time between this update and the last one in nano seconds
         *      estimateIntervalNsecs - the required interval between updates
         *      nEstimates - the number of estimates to make starting at the start time
         */
        void beginCycle(ros::Time startTime, long updateFrequencyNsecs, long estimateIntervalNsecs, int nEstimates);
        
        /**
         * Function to indicate that all joint updates have been done and the statuses should now be published
         * Params:
         *      endTime - the time the update cycle is considered to end at (the time the instructions are written to the board)
         */
        void endCycle(ros::Time endTime);
    private:
        //stores the joint controllers we need to poll for joint states
        std::vector<JointController*> joints;
        
        //stores the start and end of the most recent cycle
        ros::Time cycleStart, cycleEnd;
        //the update interval for extrapolation
        ros::Duration updateInterval;
        //the number of estimates to make
        int numEstimates;
        
        //the current joint message
        sensor_msgs::JointState currentStateMessage;
        
        //required ros handlers
        ros::NodeHandle nh;
        ros::Publisher statesPub;
        ros::Publisher debugPub;
        
        void publish_joint(std::string name, double position, double velocity, double effort, int jointNo);
        
};

#endif //JOINTS_MONITOR_H
