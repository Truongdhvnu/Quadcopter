#ifndef TYPE_H
#define TYPE_H
#include "contrains.h"

/*
    Control parameters: Roll, Pitch and Yaw
*/
class Control_paras {
public:
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    Control_paras() {}

    Control_paras(float _roll, float _pitch, float _yaw) : roll(_roll), pitch(_pitch), yaw(_yaw) {}

    Control_paras operator-(const Control_paras& other) const {
        return Control_paras(roll - other.roll, pitch - other.pitch, yaw - other.yaw);
    }

    Control_paras operator+(const Control_paras& other) const {
        return Control_paras(roll + other.roll, pitch + other.pitch, yaw + other.yaw);
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