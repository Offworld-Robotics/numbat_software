/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in velocity and outputs a pwm rate.
 */
#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H

#define SECONDS_IN_MINUTE 60
#define FLOATING_PT_ERROR 0.001
#define FLOAT_EQL(x,y) ((fabs(x - y)) > FLOATING_PT_ERROR)


class JointVelocityController {
    
    public:
        JointVelocityController(int minPWM, int maxPWM, int maxRPM, double wheelRadius);
        int velToPWM(double targetVel, double currentVel, double currentAngel);
                                
    private:
        double wheelRadius;
        int  minPWM, maxPWM;
        int maxRPM;
        int currentPWM;
        double deltaPWM;
        double deltaVelocity;
        
        double lastAngularVelocity;
        int lastPWM;
    
}

#endif