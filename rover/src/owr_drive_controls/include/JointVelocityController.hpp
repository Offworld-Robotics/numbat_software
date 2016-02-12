/*
 * Orignal Author: Harry J.E Day
 * Editors:
 * Date Started: 12/02/2016
 * Purpose: Represents an interface for controlling a motor. It takes in velocity and outputs a pwm rate.
 */

class JointVelocityController {
    
    public:
        JointVelocityController(double wheelRadius, double maxPWM, double minPWM, double motorRPM, double motorBaseVoltage, double motorAmps, double gearRatioIn);
        int velToPWM(double vel);
                                
    private:
        double wheelRadius;
        double  minPWM, maxPWM;
        double motorBaseVoltage;
        double motorAmps;
        double gearRatioIn;
    
}