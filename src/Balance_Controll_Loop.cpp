#include <Arduino.h>
#include "Rate_Controll_Loop.cpp"

class Balance_Controll_Loop
{
private:
    Rate_Controll_Loop controll_loop;   
public:
    void begin() {
        controll_loop.begin();
    }

    void balance(Angle_paras input) {
        // controll_loop.mpu6050.update();
        // Angle_paras feedback = controll_loop.mpu6050.get_gyro_paras();
        // Angle_paras plant_input = calculate_plan_inputs(input-feedback, mpu6050.getInterval());
        // motorsController.set_motors_speed(plant_input);
    }
};
