#include <Arduino.h>
#include "Rate_Controll_Loop.cpp"

class Angle_Controll_Loop
{
private:
    Rate_Controll_Loop rate_controll_loop;
    // PID_Controller roll_angle_controller = PID_Controller(2.0,0.0,0.0);
    // PID_Controller pitch_angle_controller = PID_Controller(2.0,0.0,0.0); 
    float proportion = 2.0;
public:
    void begin() {
        rate_controll_loop.begin();
    }

    void loop(Control_paras desired_input) {
        rate_controll_loop.mpu6050.update();
        // rate_controll_loop.mpu6050.get_angle_paras(): Kalman roll & pitch angle and yaw rate = 0.
        Control_paras Kalman_feedback = rate_controll_loop.mpu6050.get_angle_paras();
        Control_paras desired_rotation_rate = Control_paras(proportion*(desired_input.roll - Kalman_feedback.roll), 
                                                            proportion*(desired_input.pitch - Kalman_feedback.pitch), 
                                                            desired_input.yaw);
        Control_paras plant_input = rate_controll_loop.calculate_plan_inputs(desired_rotation_rate - 
                                                                            rate_controll_loop.mpu6050.get_gyro_paras(), 
                                                                            rate_controll_loop.mpu6050.getInterval());
        rate_controll_loop.motorsController.set_motors_speed(plant_input);
    }

#ifdef LOG
    void test_angle_controll_loop(Control_paras desired_input) {
        long timer = millis();
        int i = 0;
        while(true) {
            /*
                loop part
            */
            rate_controll_loop.mpu6050.update();
            // rate_controll_loop.mpu6050.get_angle_paras(): Kalman roll & pitch angle and yaw rate = 0.
            Control_paras Kalman_feedback = rate_controll_loop.mpu6050.get_angle_paras();
            Control_paras desired_rotation_rate = Control_paras(proportion*(desired_input.roll - Kalman_feedback.roll), 
                                                                proportion*(desired_input.pitch - Kalman_feedback.pitch), 
                                                                desired_input.yaw);
            Control_paras plant_input = rate_controll_loop.calculate_plan_inputs(desired_rotation_rate - 
                                                                                rate_controll_loop.mpu6050.get_gyro_paras(), 
                                                                                rate_controll_loop.mpu6050.getInterval());
            rate_controll_loop.motorsController.set_motors_speed(plant_input);
            /*
                logging command
            */
            // Serial.println(rate_controll_loop.mpu6050.getInterval(), 5);
            // plant_input.log();
            if (i++ % 100 == 0) {
                Kalman_feedback.log();
                rate_controll_loop.motorsController.log_motors_rate();
            }
            // Serial.println(millis()-timer);
        }
    }
#endif
};
