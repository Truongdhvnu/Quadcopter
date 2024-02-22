#include <Arduino.h>
#include "contrains.h"
#include "PID_Controller.h"

/*
    Dể là uint32_t output nó không báo lỗi nhưng khi chạy bộ PID thì tham số đo ra sai
    Bực thật sao nó ko báo lỗi nhỉ
*/
static float inline limit_pid_ouput(float ouput) {
    if (ouput < OUPUT_PID_MIN) {
        return float(OUPUT_PID_MIN);
    }
    if (ouput > OUPUT_PID_MAX) {
        return float(OUPUT_PID_MAX);
    }
    return ouput;
}

PID_Controller::PID_Controller(float P, float I, float D) {
    this->P = P;
    this->I = I;
    this->D = D;
}

void PID_Controller::set_parameters(float P, float I, float D) {
    this->P = P;
    this->I = I;
    this->D = D;
}

float PID_Controller::calculate_plan_input(Error error, float interval) {
    prevIterm += I * (prevError + error) * interval / 2;
    prevIterm = limit_pid_ouput(prevIterm);
    float result = P * error + prevIterm + D * (error - prevError) / interval;
    prevError = error;
    return limit_pid_ouput(result);
}
#ifdef LOG
String PID_Controller::print_paras() {
    char buffer[60]; // Chú ý nếu size của buffer < lượng ký tự trong sprintf thì lỗi 
    sprintf(buffer, "P: %.2f I: %.2f D: %.2f", this->P, this->I, this->D);
    return buffer;
#endif
}