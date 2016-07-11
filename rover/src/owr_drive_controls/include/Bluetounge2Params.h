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


/*
 * NOTE: The pot is a 10 turn pot, this data limits it to one turn
 */

#define SWERVE_POT_R_CENTER 512.0
#define SWERVE_POT_L_CENTER 497.0
#define SWERVE_POT_L_LIMIT_N_DEG 470.0
#define SWERVE_POT_L_LIMIT_P_DEG 523.0
#define SWERVE_POT_L_REVOLUTION 99
#define SWERVE_POT_TURNS 1
#define SWERVE_POT_R_LIMIT_N_DEG 482.0
#define SWERVE_POT_R_LIMIT_P_DEG 539.0
#define SWERVE_POT_R_REVOLUTION 100


//TODO: put values here
#define ARM_POT_CENTER 0
#define ARM_POT_LIMIT_N_DEG 0
#define ARM_POT_LIMIT_P_DEG 0
#define ARM_POT_REVOLUTION 0
#define ARM_POT_TURNS 0


#endif //BLUETOUNGE_2_PARAMS_H
