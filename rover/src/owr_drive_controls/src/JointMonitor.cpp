/*
 * Handles the monitoring of joints for transforms
 * Original Author: Harry J.E Day
 * NOTE: Joint Publisher code based on code originally by Simon in Bluetounge.cpp
 * Editors:
 * Date Started: 17/02/2016
 * ros_package: owr_drive_controls
 * ros_node: owr_board_control
 */

#include "JointsMonitor.hpp"
#include "owr_messages/board.h"
#include "owr_messages/pwm.h"

JointsMonitor::JointsMonitor(ros::NodeHandle nodeHandle) {
    nh = nodeHandle;
    statesPub =  nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    debugPub = nh.advertise<owr_messages::board>("/owr/jointsDebug", 1); //TODO: create a message type for this
    //start the message sequence at zero
    currentStateMessage.header.seq = 0;
}

void JointsMonitor::beginCycle(ros::Time startTime, long updateFrequencyNsecs, long estimateIntervalNsecs, int nEstimates) {
    cycleStart = startTime;
    updateInterval = ros::Duration(0, estimateIntervalNsecs);
    numEstimates = nEstimates;
    //TODO
}

void JointsMonitor::endCycle(ros::Time endTime) {
    cycleEnd = endTime;
    owr_messages::board statusMsg;
    statusMsg.header.stamp = endTime;
    ros::Time estimateTime = endTime;
    int i,j;
    currentStateMessage.velocity.resize(joints.size());
    currentStateMessage.position.resize(joints.size());
    currentStateMessage.effort.resize(joints.size());
    currentStateMessage.name.resize(joints.size());
    for(i =0; i < numEstimates; i++, estimateTime+=updateInterval) {
        currentStateMessage.header.stamp = estimateTime;
        currentStateMessage.header.seq +=1;
        j =0;
        for(std::vector<JointController*>::iterator it = joints.begin(); it != joints.end(); ++it, j++) {
            jointInfo info = (*it)->extrapolateStatus(cycleStart, estimateTime);
            publish_joint(info.jointName, info.position, info.velocity, info.effort, j);

            //TODO: add to debug message
        }
        statesPub.publish(currentStateMessage);
    }
    for(std::vector<JointController*>::iterator it = joints.begin(); it != joints.end(); ++it, j++) {
            jointInfo info = (*it)->extrapolateStatus(cycleStart, endTime);
	    owr_messages::pwm pwmMsg;
	    pwmMsg.joint = info.jointName;
	    pwmMsg.pwm = info.pwm;
	    pwmMsg.currentVel = info.velocity;
	    pwmMsg.currentPos = info.position;
            statusMsg.joints.push_back(pwmMsg);

            //TODO: add to debug message
    }
    debugPub.publish(statusMsg);  
    
    
}

void JointsMonitor::addJoint(JointController * jc) {
    joints.push_back(jc);
}
    
    
void JointsMonitor::publish_joint(std::string name, double position, double velocity, double effort, int jointNo){
    currentStateMessage.name[jointNo] = name;
    currentStateMessage.position[jointNo] = position;
    currentStateMessage.velocity[jointNo] = velocity;
    currentStateMessage.effort[jointNo] = effort;
}