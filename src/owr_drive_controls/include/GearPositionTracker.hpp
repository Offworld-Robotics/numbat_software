/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 20/02/2016
 * Purpose: Class to convert encoder speeds to gears
 */

#include <PositionTracker.hpp>

class GearPositionTracker : public PositionTracker {
    public:
        GearPositionTracker(const std::vector< double > & gears);
        GearPositionTracker(const double * gears,const int nGears);
        void updatePos(double speed);
        void updatePos(double speed, ros::Time current);
    private:
        const std::vector< double > gears;
};
