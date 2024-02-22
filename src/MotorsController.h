#ifndef MOTORSCONTROLLER_H
#define MOTORSCONTROLLER_H

#include <Arduino.h>
#include "type.h"

class Motorscontroller{
private:
    uint32_t ul_rate = 0;
    uint32_t ur_rate = 0;
    uint32_t dl_rate = 0;
    uint32_t dr_rate = 0;
public:
    void initial_pwm(int pin, int channel);

    void initial_pwms();
	
    void test_initial_pwms();
    
    void set_motors_speed(Control_paras motor_power_commamd);
    
    void set_rates();

#ifdef LOG
    void log_motors_rate();
#endif
};

#endif