#ifndef BLUETOUNGE_2_PARAMS_H
#define BLUETOUNGE_2_PARAMS_H

const double WHEEL_GEARS[] = {0.01}; 
// const std::vector < double > SWERVE_GEARS_VECTOR = {0.1, 0.2};
#define WHEEL_N_GEARS 1
#define WHEEL_MOTOR_MAX_PWM 1750
#define WHEEL_MOTOR_MIN_PWM 1250
#define WHEEL_MOTOR_RPM 14000 //TODO: confirm this
#define WHEEL_RADIUS 0.12 //12mm

//this one has to be const because array
const double SWERVE_GEARS[] = {1.0/455.0, -1.0/4.75};
// const std::vector < double > SWERVE_GEARS_VECTOR = {0.1, 0.2};
#define SWERVE_N_GEARS 2
#define SWERVE_MOTOR_MAX_PWM 2000
#define SWERVE_MOTOR_MIN_PWM 1000
#define SWERVE_MOTOR_RPM (26*455) 
#define SWERVE_RADIUS 0.12 //TODO

const double ARM_BASE_ROTATE_GEARS[] = {1.0/455.0, 1}; //TODO: put not stupid values here
// const std::vector < double > ARM_BASE_GEARS_VECTOR {0.1};
#define ARM_BASE_ROTATE_N_GEARS 2
#define ARM_BASE_ROTATE_MOTOR_MAX_PWM 2000
#define ARM_BASE_ROTATE_MOTOR_MIN_PWM 1000
#define ARM_BASE_ROTATE_MOTOR_RPM (26*455) 
#define ARM_BASE_ROTATE_RADIUS 0.01 //TODO: get this
#define ARM_INCE_RATE_MULTIPLIER 0.1

//TODO: set these values, currently intentionally crazy so that it is noticable they are wrong
#define SWERVE_POT_MIN -10000
#define SWERVE_POT_MAX 100000
#define SWERVE_POT_TURNS 10


#define ARM_POT_MIN SWERVE_POT_MIN 
#define ARM_POT_MAX SWERVE_POT_MAX
#define ARM_POT_TURNS SWERVE_POT_TURNS


#endif //BLUETOUNGE_2_PARAMS_H
