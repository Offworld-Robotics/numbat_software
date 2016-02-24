/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointVelocityController.hpp"

class JointSpeedBasedPositionController : public JointController {
    public:
        JointSpeedBasedPositionController(double radius, const double * gearRatio, int nGears,int minPWM, int maxPWM, int maxRPM, char * topic, ros::NodeHandle nh, std::string name);
        int posToPWM(double futurePos, double currentPos, double updateFrequency);
        int posToPWM(double currentPos, double updateFrequency);
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime);
    private:
        double radius;
        const double * gearRatio;
        int nGears;
        int minPWM, maxPWM;
        int maxRPM;
        
        int currentPWM;
        int deltaPWM;
        
        double maxVelocity;
        double velocityRange;
        double lastTargetVelocity;
        double lastAngularVelocity;
        
        double lastAimPosition, lastKnownPosition;
        int lastPWM;
        double lastDeltaT;
    
};