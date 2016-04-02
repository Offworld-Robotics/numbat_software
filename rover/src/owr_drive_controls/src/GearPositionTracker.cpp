/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 20/02/2016
 * Purpose: Class to convert encoder speeds to gears
 */

#include <math.h>

#include "GearPositionTracker.hpp"

GearPositionTracker::GearPositionTracker ( const std::vector< double > & gearsIn ) : gears (gearsIn) {
}

GearPositionTracker::GearPositionTracker (const double* gears,const int nGears ) : gears(gears, gears + nGears) {
}


void GearPositionTracker::updatePos ( double speed ) {
    ros::Time current = ros::Time::now();
    updatePos(speed, current);
}

void GearPositionTracker::updatePos(double speed, ros::Time current) {
    //TODO: do error checking, etc
    mutext.lock();
    if (positions.size() != 0) {
        double distance;
        if(positions.size() == 1) {
            distance = (speed * (current - times.back()).toSec());
        }  else {
            distance = ((speed+velocities.back())/2.0 * (current - times.back()).toSec());
        }
        //do gear ratios
        for(int i = 0; i < gears.size(); i++) {
            distance *= gears[i];
        }
        double pos = positions.back() + distance;
//         while (pos < 0) {
//             pos = 2*M_PI + pos;
//         }
//         pos = fmod(pos, 2*M_PI);
        while (pos > M_PI) {
            pos = -(2*M_PI - pos);
        } 
        while (pos < -M_PI) {
           pos = 2*M_PI + pos;
        }
        positions.push_back(pos);
        times.push_back(current);
        velocities.push_back(speed);
    } else {
        velocities.push_back(speed);
        positions.push_back(0.0);
        times.push_back(current);
    }
    mutext.unlock();
}

double GearPositionTracker::getPosition () {
    //TODO: move based on time
    return positions.back();
}


void GearPositionTracker::resetPos() {
    mutext.lock();
    positions.clear();
    velocities.clear();
    times.clear();
    mutext.unlock();
}
