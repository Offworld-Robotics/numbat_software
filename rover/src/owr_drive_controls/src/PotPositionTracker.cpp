/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 29/06/2016
 * Purpose: Class to convert pot values to a position 
 */

#include "PotPositionTracker.hpp"
#include <cmath>

PotPositionTracker::PotPositionTracker(int maxV, int minV, int turns) : maxValue(maxV), minValue(minV), turns(turns) {

}

void PotPositionTracker::updatePos(double potValue) {
    updatePos(potValue,ros::Time::now());
}

void PotPositionTracker::updatePos(double potValue, ros::Time current) {
    double pos = fmod(((potValue - minValue) / (maxValue - minValue)) * (2.0 * M_PI * turns),(2.0 * M_PI));
    //TODO: make this calibratable
    //TODO: calc velocity
    
    updateLists(pos, 0.0, current,true);
    
}



