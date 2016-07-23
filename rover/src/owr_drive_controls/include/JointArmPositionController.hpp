/*
 * Orignal Author: Simon Ireland
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in a target position, and current postion and outputs a pwm rate.
 */

class JointArmPositionController : public JointController {
    public:
        JointArmPositionController(int minPosition, int maxPosition, int minValue, int maxValue, int minPWM, int maxPWM, char * topic, ros::NodeHandle nh, std::string name);
        int posToPWM(int currentPos, int futurePos, double updateFrequency);
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime);
    private:
        int minPosition, maxPosition;
        
        int minValue, maxValue;
        
        int minPWM, maxPWM
        
        int lastPos, lastPWM;
};
