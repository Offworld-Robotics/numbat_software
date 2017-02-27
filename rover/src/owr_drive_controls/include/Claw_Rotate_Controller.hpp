/*
 * Orignal Author: Harry J.E Day
 * Date Started: 27/02/17
 * Purpose: Used to drive the claw. This is very hacky, dosen't really do what it says
 * Copyright BLUEsat UNSW 2017. Released under the MIT License.
 */

#include "JointController.hpp"

class Claw_Rotate_Controller : public JointController {
    public:
        Claw_Rotate_Controller(int max_PWM, int min_PWM, int * trim, char * topic, ros::NodeHandle nh, std::string name);
        int velToPWM(int futureVel);
        int velToPWM();
        virtual jointInfo extrapolateStatus(ros::Time sessionStart, ros::Time extrapolationTime);
    private:
        
        int minPWM, maxPWM;

        double lastPos, lastPWM;

        int * trim;
};
