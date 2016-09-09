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
#define ARM_BASE_ROTATE_MOTOR_MAX_PWM 1800
#define ARM_BASE_ROTATE_MOTOR_MIN_PWM 1200
#define ARM_BASE_ROTATE_MOTOR_RPM (26*455) 
#define ARM_BASE_ROTATE_RADIUS 0.01 //TODO: get this
#define ARM_INCE_RATE_MULTIPLIER 0.05
#define ARM_ROTATE_RATE 0.01


/*
 * NOTE: The pot is a 10 turn pot, this data limits it to one turn
 */

#define SWERVE_POT_R_CENTER 351.0//2069.0//1961.0 //2088.0 
#define SWERVE_POT_L_CENTER 341.0//2015.0 //2032.0
#define SWERVE_POT_L_LIMIT_N_DEG 321.0//1890.0 //1932.0 
#define SWERVE_POT_L_LIMIT_P_DEG 361.0 //2131.0 
#define SWERVE_POT_L_REVOLUTION 66
#define SWERVE_POT_TURNS 1
#define SWERVE_POT_R_LIMIT_N_DEG 332.0//1958.0 //1988.0
#define SWERVE_POT_R_LIMIT_P_DEG 370.0//2188.0
#define SWERVE_POT_R_REVOLUTION 66
#define SWERVE_ACCURACY 0.2


// Nuno and Denis did these lololololol!
#define ARM_POT_CENTER 1963
#define ARM_POT_LIMIT_N_DEG 1160
#define ARM_POT_LIMIT_P_DEG 2783
#define ARM_POT_REVOLUTION 820
#define ARM_POT_TURNS 22 
#define ARM_ACCURACY 0.0

#define ARM_ACT_PWM_MIN 1000
#define ARM_ACT_PWM_MAX 2000

// Placeholders, not currently used
// These [MIN, MAX] are not limits, but are instead used for mapping ADC reading to length measurement.
#define LOWER_MIN_POS 0.07
#define LOWER_MAX_POS 0.22
#define LOWER_MIN_ADC 3640
#define LOWER_MAX_ADC 1457

#define UPPER_MIN_POS 0.04
#define UPPER_MAX_POS 0.16
#define UPPER_MIN_ADC 610
#define UPPER_MAX_ADC 2804


#endif //BLUETOUNGE_2_PARAMS_H
