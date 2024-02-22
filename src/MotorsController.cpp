#include "MotorsController.h"
#include "contrains.h"

/*
	1. PWM with ESP32
		- 16 channel PWM (0-15)
		- Frequency of PWM (up to 40 MHz) (which frequency should be chosen?)
		- Resolution (1->16 bit)
*/

void Motorscontroller::initial_pwm(int pin, int channel) {
    pinMode(pin, OUTPUT);
    ledcSetup(channel, MOTOR_FREQ, MOTOR_RESOLUTION);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);
}

void Motorscontroller::initial_pwms() {
    initial_pwm(MOTOR_UR_PIN, MOTOR_UR_CHANEL);
    initial_pwm(MOTOR_UL_PIN, MOTOR_UL_CHANEL);
    initial_pwm(MOTOR_DL_PIN, MOTOR_DL_CHANEL);
    initial_pwm(MOTOR_DR_PIN, MOTOR_DR_CHANEL);
}

static uint32_t inline limit_rate(uint32_t rate) {
    if (rate < MOTOR_THROTTLE_MIN) {
        return uint32_t(MOTOR_THROTTLE_MIN);
    }
    if (rate > MOTOR_THROTTLE_MAX) {
        return uint32_t(MOTOR_THROTTLE_MAX);
    }
    return rate;
}

/* 
    Để điều khiển độ cao cụ thể, cần thêm 1 cảm biến đo độ cao nữa, và 1 PID_contoller nữa cho input throttle
    Hiện tại chưa code được chức năng đó nên để mặc định input throttle là 45%  
    motor1: dr motor
    motor2: dl motor
    motor3: ul motor
    motor4: ur motor
*/
void Motorscontroller::set_motors_speed(Control_paras motor_power_commamd) { 
    dr_rate = uint32_t(DEFAUTL_THROTTEL - motor_power_commamd.roll - motor_power_commamd.pitch - motor_power_commamd.yaw);
    dl_rate = uint32_t(DEFAUTL_THROTTEL - motor_power_commamd.roll + motor_power_commamd.pitch + motor_power_commamd.yaw);
    ul_rate = uint32_t(DEFAUTL_THROTTEL + motor_power_commamd.roll + motor_power_commamd.pitch - motor_power_commamd.yaw);
    ur_rate = uint32_t(DEFAUTL_THROTTEL + motor_power_commamd.roll - motor_power_commamd.pitch + motor_power_commamd.yaw);

    /*
        Apply contrains for motors's rate
    */
    dr_rate = limit_rate(dr_rate);
    dl_rate = limit_rate(dl_rate);
    ul_rate = limit_rate(ul_rate);
    ur_rate = limit_rate(ur_rate);

    set_rates();
}

void Motorscontroller::set_rates() {
    ledcWrite(MOTOR_UR_CHANEL, ur_rate);
    ledcWrite(MOTOR_DR_CHANEL, dr_rate);
    ledcWrite(MOTOR_DL_CHANEL, dl_rate);
    ledcWrite(MOTOR_UL_CHANEL, ul_rate);
}

#ifdef LOG
void Motorscontroller::log_motors_rate() {
    char buffer[60]; // Chú ý nếu size của buffer < lượng ký tự trong sprintf thì lỗi 
    sprintf(buffer, "dr: %.4d dl: %.4d ul: %.4d ur: %.4d", dr_rate, dl_rate, ul_rate, ur_rate);
    Serial.println(buffer);
}
#endif

void Motorscontroller::test_initial_pwms() {
    /*
        Tets pwm for UL motor
    */
    for(int i = 0; i <= MOTOR_THROTTLE_MAX; i++) {
        ledcWrite(MOTOR_UL_CHANEL, i);
        delay(4);
    }
    for(int i = MOTOR_THROTTLE_MAX; i >= 0; i--) {
        ledcWrite(MOTOR_UL_CHANEL, i);
        delay(4);
    }
    /*
        Tets pwm for UR motor
    */
    for(int i = 0; i <= MOTOR_THROTTLE_MAX; i++) {
        ledcWrite(MOTOR_UR_CHANEL, i);
        delay(4);
    }
    for(int i = MOTOR_THROTTLE_MAX; i >= 0; i--) {
        ledcWrite(MOTOR_UR_CHANEL, i);
        delay(4);
    }
    /*
        Tets pwm for DL motor
    */
    for(int i = 0; i <= MOTOR_THROTTLE_MAX; i++) {
        ledcWrite(MOTOR_DL_CHANEL, i);
        delay(4);
    }
    for(int i = MOTOR_THROTTLE_MAX; i >= 0; i--) {
        ledcWrite(MOTOR_DL_CHANEL, i);
        delay(4);
    }
    /*
        Tets pwm for DR motor
    */
    for(int i = 0; i <= MOTOR_THROTTLE_MAX; i++) {
        ledcWrite(MOTOR_DR_CHANEL, i);
        delay(4);
    }
    for(int i = MOTOR_THROTTLE_MAX; i >= 0; i--) {
        ledcWrite(MOTOR_DR_CHANEL, i);
        delay(4);
    }
}