/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 20/02/2016
 * Purpose: Class to convert encoder speeds to gears
 */

#include <ros/ros.h>

class GearPositionTracker {
    public:
        GearPositionTracker(std::vector< double > gears);
        GearPositionTracker(const double * gears,const int nGears);
        void updatePos(double speed);
        double getPosition();
    private:
        std::vector< const double > gears;
        std::vector < double > velocities;
        std::vector < double > positions;
        std::vector < ros::Time > times;
};
