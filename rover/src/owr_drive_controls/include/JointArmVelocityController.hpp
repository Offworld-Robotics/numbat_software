/*
 * Orignal Author: Simon Ireland
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

#include "JointVelocityController.hpp"

class JointArmVelocityController : public JointController {
    public:
        JointArmVelocityController(int minPWM, int maxPWM, char * topic, ros::NodeHandle nh, std::string name);
        int velToPWM(int futureVel);
        void updatePos(int positionIn);
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime);
    private:
        
        int minPWM, maxPWM;
        
        int lastPos, lastPWM;
};
