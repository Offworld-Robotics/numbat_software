/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 29/06/2016
 * Purpose: Class to convert pot values to a position 
 */

#include <PositionTracker.hpp>

class PotPositionTracker : public PositionTracker {
    public:
        PotPositionTracker(int maxV, int minV, int turns);
        void updatePos(double potValue);
        void updatePos(double potValue, ros::Time current);
        virtual void resetPos();
    private:
        const std::vector< double > gears;
        int maxValue, minValue, turns;
        int center;
};
