#ifndef TYPE_H
#define TYPE_H
#include "contrains.h"

class Rate_paras {
public:
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    Rate_paras() {}

    Rate_paras(float _roll, float _pitch, float _yaw) : roll(_roll), pitch(_pitch), yaw(_yaw) {}

    Rate_paras operator-(const Rate_paras& other) const {
        return Rate_paras(roll - other.roll, pitch - other.pitch, yaw - other.yaw);
    }

    Rate_paras operator+(const Rate_paras& other) const {
        return Rate_paras(roll + other.roll, pitch + other.pitch, yaw + other.yaw);
    }

#ifdef LOG
    void log() {
        char buffer[60];
        sprintf(buffer, "roll: %.2f pitch: %.2f yaw: %.2f", roll, pitch, yaw);
        Serial.println(buffer);
    }
#endif
};

class Angle_paras {
public:
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    Angle_paras() {}

    Angle_paras(float _roll, float _pitch, float _yaw) : roll(_roll), pitch(_pitch), yaw(_yaw) {}

    Rate_paras operator-(const Rate_paras& other) const {
        return Rate_paras(roll - other.roll, pitch - other.pitch, yaw - other.yaw);
    }

    Rate_paras operator+(const Rate_paras& other) const {
        return Rate_paras(roll + other.roll, pitch + other.pitch, yaw + other.yaw);
    }

#ifdef LOG
    void log() {
        char buffer[60];
        sprintf(buffer, "roll: %.2f pitch: %.2f yaw: %.2f", roll, pitch, yaw);
        Serial.println(buffer);
    }
#endif
};

#endif