/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 20/02/2016
 * Purpose: Class to convert encoder speeds to gears
 */

#include <ros/ros.h>
#include <ecl/threads.hpp>

class GearPositionTracker {
    public:
        GearPositionTracker(const std::vector< double > & gears);
        GearPositionTracker(const double * gears,const int nGears);
        void updatePos(double speed);
        void updatePos(double speed, ros::Time current);
        void resetPos();
        double getPosition();
    private:
        const std::vector< double > gears;
        std::vector < double > velocities;
        std::vector < double > positions;
        std::vector < ros::Time > times;
        ecl::Mutex mutext;
};
