/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 29/06/2016
 * Purpose: Class to convert pot values to a position 
 */

#include "PotPositionTracker.hpp"
#include <cmath>
#include <limits>
#include <ros/ros.h>

#define TURN_SAFTEY_MARGIN 1.0

PotPositionTracker::PotPositionTracker(int maxV, int minV, int turns) : maxValue(maxV), minValue(minV), turns(turns) {
    center = (maxV + minV)/2;
}

void PotPositionTracker::updatePos(double potValue) {
    updatePos(potValue,ros::Time::now());
}

void PotPositionTracker::updatePos(double potValue, ros::Time current) {
    //put the pot value in the range -PI*turns to PI*turns
    //TODO: check that this works when the center value is not the center, I'm pretty sure it does
    double pos = ((potValue - center) / (maxValue - minValue)) * (2.0 * M_PI * turns);
    
    //check we haven't overshot
    if(
        (pos > 2.0 * M_PI * (turns - TURN_SAFTEY_MARGIN)) || 
        (pos < 2.0 * M_PI * (turns - TURN_SAFTEY_MARGIN))
    ) {
        pos = std::numeric_limits<double>::quiet_NaN();
        ROS_ERROR("Too many turns! Position is %f, pot value is %f", pos, potValue);
    }
        
    
    
    //make sure the pos is in our 'nice' range (between M_PI and -M_PI)
    while (pos > M_PI) {
        pos = -(2*M_PI - pos);
    } 
    while (pos < -M_PI) {
        pos = 2*M_PI + pos;
    }
    //TODO: calc velocity
    
    updateLists(pos, 0.0, current,true);
    
}


void PotPositionTracker::resetPos() {
    mutext.lock();
    if(positions.size() > 0) {
        //TODO: account for velocity
        center = positions.back();
    } else {
        center = maxValue - minValue;
    }
    mutext.unlock();
    PositionTracker::resetPos();
}

