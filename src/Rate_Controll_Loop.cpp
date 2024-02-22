#include <Arduino.h>
#include "contrains.h"
#include "PID_Controller.h"
#include "MotorsController.h"
#include "type.h"
#include "MPU6050.h"
#include "Wire.h"

/* 
    Để điều khiển độ cao, cần thêm 1 cảm biến đo độ cao nữa, và 1 PID_controller nữa cho input throttle
    Hiện tại chưa code được chức năng đó nên để mặc định input throttle là 45%  
*/
class Rate_Controll_Loop {
// private:
public:
    float desired_roll_rate = 0;
    float desired_pitch_rate = 0;
    float desired_yaw_rate = 0;
    float measured_roll_rate = 0;
    float measured_pitch_rate = 0;
    float measured_yaw_rate = 0;
    PID_Controller roll_rate_controller;
    PID_Controller pitch_rate_controller;
    // PID_Controller yaw_rate_controller(2.0,12.0,0.0); // why this error (this syntax not error in program cpp in PC)
    PID_Controller yaw_rate_controller = PID_Controller(2.0,12.0,0.0);
public:
    MPU6050 mpu6050 = MPU6050(Wire);
    Motorscontroller motorsController;

    void begin() {
        motorsController.initial_pwms();
	    mpu6050.begin();
    }

    void set_desired_rates(float roll, float pitch, float yaw) {
        this->desired_roll_rate = roll;
        this->desired_pitch_rate = pitch;
        this->desired_yaw_rate = yaw;
    }

    void get_feedback_rates(float roll, float pitch, float yaw) {
        this->measured_roll_rate = roll;
        this->measured_pitch_rate = pitch;
        this->measured_yaw_rate = yaw;
    }

    /*
        Remember that controller outputs are inputs of plant (motors_controller) 
    */
    Rate_paras calculate_plan_inputs(Rate_paras error, float interval) {
        Rate_paras controller_ouput;
        controller_ouput.roll = roll_rate_controller.calculate_plan_input(error.roll, interval);
        controller_ouput.pitch = pitch_rate_controller.calculate_plan_input(error.pitch, interval);
        controller_ouput.yaw = yaw_rate_controller.calculate_plan_input(error.yaw, interval);
        return controller_ouput;
    }

    /*
        !Attention: When logging so much data to Serial, time interval becomes unstable? why?
        In this case, It occurs when using 2 or 3 of log commands below and don't when using only one.
    */
    void test_rate_controll_loop(Rate_paras desired_rate) {
        long timer = millis();
        int i = 0;
        while(true) {
            mpu6050.update();
            Rate_paras feedback = mpu6050.get_gyro_paras();
            Rate_paras plant_input = calculate_plan_inputs(desired_rate-feedback, mpu6050.getInterval());
            motorsController.set_motors_speed(plant_input);
            /*
                logging command
            */
            // Serial.println(mpu6050.getInterval(), 5);
            // feedback.log();
            // plant_input.log();
            if (i++ % 100 == 0) {
                motorsController.log_motors_rate();
            }
            // Serial.println(millis()-timer);
        }
    }

#ifdef LOG
    void log_pid_parameters() {
        Serial.println(roll_rate_controller.print_paras());
        Serial.println(pitch_rate_controller.print_paras());
        Serial.println(yaw_rate_controller.print_paras());
    }
#endif

};