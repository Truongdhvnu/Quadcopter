#ifndef CONTRAINS_H
#define CONTRAINS_H

/*
    When logging so much data to Serial, time interval becomes un stable? why?
*/
#define LOG
#define BAUDRATE                    250000

/*
    Motor contrains
*/
#define MOTOR_UL_PIN                4    // UL : up left
#define MOTOR_UL_CHANEL             0 
#define MOTOR_UR_PIN                14   // UR : up right
#define MOTOR_UR_CHANEL             1
#define MOTOR_DL_PIN                19
#define MOTOR_DL_CHANEL             2
#define MOTOR_DR_PIN                27
#define MOTOR_DR_CHANEL             3
#define MOTOR_FREQ                  250
#define MOTOR_RESOLUTION            10
#define MOTOR_THROTTLE_MAX          800
#define MOTOR_THROTTLE_MIN          (MOTOR_THROTTLE_MAX*0.25)
#define DEFAUTL_THROTTEL            (MOTOR_THROTTLE_MAX*0.55)

/*
    MPU6050 contrains
*/
#define I2C_MPU6050_RATE            100000
#define I2C_MPU6050_SDA             21
#define I2C_MPU6050_SCL             22
// stupid variable using to calculating angles by gyro only
// used only to show the uses of the kalman filter only 
// not use this parameter final code
#define stupid

/*
    PID controller contrains
*/
#define OUPUT_PID_MAX (0.45*MOTOR_THROTTLE_MAX)
#define OUPUT_PID_MIN (-0.45*MOTOR_THROTTLE_MAX)

#define Ts 0.004
#define Error float
#endif