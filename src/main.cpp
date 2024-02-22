/*
	Development kit: DOIT ESP32 DEVKIT V1
*/
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "MotorsController.h"
#include "contrains.h"
#include "type.h"
#include "Rate_Controll_loop.cpp"

Rate_Controll_Loop controll_loop;
void setup() {
#ifdef LOG
	Serial.begin(BAUDRATE);
#endif
	controll_loop.begin();
}
int i = 0;
void loop() {
	Rate_paras desired_rate(0.0,0.0,0.0);
	controll_loop.test_rate_controll_loop(desired_rate);
}
