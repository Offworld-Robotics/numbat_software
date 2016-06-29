/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 29/06/2016
 * Purpose: Class to manage tracking the position of a joint 
 */

#include <ros/ros.h>
#include <ecl/threads.hpp>

class PositionTracker {
    public:
        void resetPos();
        double getPosition();
    protected:
        std::vector < double > velocities;
        std::vector < double > positions;
        std::vector < ros::Time > times;
        ecl::Mutex mutext;
        
        void updateLists(double pos, double vel, ros::Time time, bool lock);
};
