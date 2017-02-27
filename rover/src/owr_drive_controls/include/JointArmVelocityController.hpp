/*
 * Orignal Author: Simon Ireland
 * Editors: Harry J.E Day
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 * Copyright BLUEsat UNSW 2016, 2017. Released under the MIT License.
 */

#include "JointVelocityController.hpp"

class JointArmVelocityController : public JointController {
    public:
        JointArmVelocityController(double minADCIn, double maxADCIn, double minPos, double maxPos ,int minPWM, int maxPWM, char * topic, ros::NodeHandle nh, std::string name);
        int velToPWM(int futureVel);
        void updatePos(int positionIn);
        int velToPWM();
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime);
    private:
        
        double minPosition, maxPosition;
        
        int minPWM, maxPWM;
        
        double minADC, maxADC;
        
        double lastPos, lastPWM;
};
