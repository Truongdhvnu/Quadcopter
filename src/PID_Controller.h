#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <string>

class PID_Controller{
private:
    Error prevError = 0;
    // Error error = 0;
    Error prevIterm = 0;
    float P = 0.6;
    float I = 3.5;
    float D = 0.03;
public:
    PID_Controller(float P = 0.6, float I = 3.5, float D = 0.03);
    void set_parameters(float P, float I, float D);
    float calculate_plan_input(Error error, float interval);
#ifdef LOG
    String print_paras();
#endif
};

#endif