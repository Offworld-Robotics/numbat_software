/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 29/06/2016
 * Purpose: Class to convert pot values to a position 
 */

#include <PositionTracker.hpp>

class PotPositionTracker : public PositionTracker {
    public:
        PotPositionTracker(float negLimit, float posLimit, float singleRotationRange, int turns, float center);
        void updatePos(double potValue);
        void updatePos(double potValue, ros::Time current);
        
        double getMinAngle();
        double getMaxAngle();
        
        virtual void resetPos();
    private:
        const std::vector< double > gears;
        float negLimit, posLimit, singleRotationRange;
        int turns;
        float center;
};
