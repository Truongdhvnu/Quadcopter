#ifndef MPU6050_H
#define MPU6050_H

#include "Wire.h"
#include <Arduino.h>
#include "type.h"

#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42
#define GYRO_XOUT_H_REG      0x43

#define GYRO_SCALE_OPTION  0x08   // FS_SEL = 500 do / s 
#define GYRO_SCALE_VALUE 65.5   // -> scale = 65.5
/*
        stupid variable using to calculating angles by gyro only
        used only to show the uses of the kalman filter only
        not use this parameter final code
*/
#define stupid

class MPU6050{
    // private:
public:
    TwoWire *wire;

    // Raw datas read from sensors (before being devided)
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    // Data after being devided for normalization parameter
    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
    // Offset of gyroScope
    float gyroXoffset, gyroYoffset, gyroZoffset;

    /*
        Kalman filter values    
    */
    // Angle values  accelerometer's values (trigonometric)
    float angleAccX, angleAccY, angleAccZ;
    // Angle values  gyroscope's values (intergration)
    float angleGyroX; float angleGyroY; float angleGyroZ;
    // Angles calculated by kalman filter
    float angleX = 0, angleY = 0, angleZ = 0;
    // The time interval between two angle calculations using the Kalman filter
    float interval;
    // milis() get paras
    long preInterval;
    // Uncertainty values for Kalman updating functions
    float K_uncertainty_roll = 4;
    float K_uncertainty_pitch = 4;
public:
    MPU6050();  
    MPU6050(TwoWire &w);
    void begin();
    void writeMPU6050(byte reg, byte data);
    byte readMPU6050(byte reg);
    void update();
    void angleIntergrate();
    void setInterval(float interV);
    void setGyroOffsets(float x, float y, float z);
    void calcGyroOffsets(bool verbose = false, uint16_t delayBefore = 5000, uint16_t delayAfter = 1000);
    /*
        This is a blocking function
        Log Pitch angle calculated by trigonometric, intergration & Kalman filter
    */
    void test_Kalman_filter(bool y_axis = true, bool x_axis = false);

    int16_t getRawAccX(){ return rawAccX; };
    int16_t getRawAccY(){ return rawAccY; };
    int16_t getRawAccZ(){ return rawAccZ; };
    int16_t getRawTemp(){ return rawTemp; };
    int16_t getRawGyroX(){ return rawGyroX; };
    int16_t getRawGyroY(){ return rawGyroY; };
    int16_t getRawGyroZ(){ return rawGyroZ; };

    Rate_paras get_gyro_paras();
    float getInterval();
    float getTemp(){ return temp; };
    float getAccX(){ return accX; };
    float getAccY(){ return accY; };
    float getAccZ(){ return accZ; };
    float getGyroX(){ return gyroX; };
    float getGyroY(){ return gyroY; };
    float getGyroZ(){ return gyroZ; };
    float getGyroXoffset(){ return gyroXoffset; };
    float getGyroYoffset(){ return gyroYoffset; };
    float getGyroZoffset(){ return gyroZoffset; };
    float getAccAngleX(){ return angleAccX; };
    float getAccAngleY(){ return angleAccY; };
    float getAngleX(){ return angleX; };
    float getAngleY(){ return angleY; };
    float getAngleZ(){ return angleZ; };
};

#endif
