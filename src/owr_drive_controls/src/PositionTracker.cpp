/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 29/06/2016
 * Purpose: Class to manage tracking the position of a joint 
 */
#include <math.h>

#include "PositionTracker.hpp"


double PositionTracker::getPosition () {
    //TODO: move based on time
    return positions.back();
}


void PositionTracker::resetPos() {
    mutext.lock();
    positions.clear();
    velocities.clear();
    times.clear();
    mutext.unlock();
}


void PositionTracker::updateLists(double pos, double vel, ros::Time time, bool lock) {
    if(lock) {
        //WARNING: mutex is not renterable
        mutext.lock();
    }
    
    positions.push_back(pos);
    times.push_back(time);
    velocities.push_back(vel);

    
    if(lock) {
        mutext.unlock();
    }
}
