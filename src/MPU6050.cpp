#include "MPU6050.h"
#include "contrains.h"

/*
	Mình gặp lỗi này khi cố gắng đọc từ slave: i2cRead returned Error 263 (ESP with arduino framework)
	Ban đầu tưởng sai địa chỉ slave, hóa ra là cái module có vấn để (về nguồn?)
	Lần sau nhớ check I2C scanner và tinh ý hơn (rõ ràng thấy 2 cái module 1 cái sáng 1 cái nháy mà mình ko chịu check)
*/

MPU6050::MPU6050(){}

MPU6050::MPU6050(TwoWire &w){
  	wire = &w;
	preInterval = millis();
	w.begin(I2C_MPU6050_SDA, I2C_MPU6050_SCL, I2C_MPU6050_RATE);
}

void MPU6050::begin(){
	// Initial MPU6050
	while (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
		delay(1000);
		Serial.print("Trying to reach MPU6050");
	};
	writeMPU6050(MPU6050_PWR_MGMT_1, 0x00); // set to zero (wakes up the MPU−6050)
	writeMPU6050(MPU6050_CONFIG, 0x05); // LPF with bandwith = 10 Hz
	writeMPU6050(MPU6050_SMPLRT_DIV, 0x07); // 8k/8 = 1k(Hz)
	writeMPU6050(MPU6050_GYRO_CONFIG, GYRO_SCALE_OPTION);
	writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);

	calcGyroOffsets(true);

	// Initial acc & gyro values
	update();

	angleGyroX = angleAccX;
	angleGyroY = angleAccY;
	angleGyroZ = 0;
	// Kalman filter values at t = 0 equal to values calculated by acceloremeter
	angleX = angleAccX;
	angleY = angleAccY;
}

void MPU6050::calcGyroOffsets(bool verbose, uint16_t delayBefore, uint16_t delayAfter){
	int32_t x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

	delay(delayBefore);

	if(verbose){
		Serial.print("Calculating gyro offsets.");
	}

	int num_rand = 1200;
	for(int i = 0; i < num_rand; i++){
		if(verbose && i % 500 == 0){
		Serial.print(".");
		}
		wire->beginTransmission(MPU6050_ADDR);
		wire->write(0x43);
		wire->endTransmission(false);
		wire->requestFrom((int)MPU6050_ADDR, 6);

		rx = wire->read() << 8 | wire->read();
		ry = wire->read() << 8 | wire->read();
		rz = wire->read() << 8 | wire->read();

		x += int32_t(rx);
		y += int32_t(ry);
		z += int32_t(rz);
		delay(4);
	}
	setGyroOffsets((x/GYRO_SCALE_VALUE)/num_rand, (y/GYRO_SCALE_VALUE)/num_rand, (z/GYRO_SCALE_VALUE)/num_rand);

	if(verbose){
	Serial.println("\nDone! Start after 1s..");
		delay(delayAfter);
	}
}

void MPU6050::angleIntergrate() {
	angleGyroX += gyroX * interval;
	angleGyroY += gyroY * interval;
	angleGyroZ += gyroZ * interval;
}

void MPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

	rawAccX = wire->read() << 8 | wire->read();
	rawAccY = wire->read() << 8 | wire->read();
	rawAccZ = wire->read() << 8 | wire->read();
	rawTemp = wire->read() << 8 | wire->read();
	rawGyroX = wire->read() << 8 | wire->read();
	rawGyroY = wire->read() << 8 | wire->read();
	rawGyroZ = wire->read() << 8 | wire->read();

	// temp = (rawTemp + 12412.0) / 340.0;

	accX = ((float)rawAccX) / 16384.0;
	accY = ((float)rawAccY) / 16384.0;
	accZ = ((float)rawAccZ) / 16384.0;

	gyroX = ((float)rawGyroX) / GYRO_SCALE_VALUE;
	gyroY = ((float)rawGyroY) / GYRO_SCALE_VALUE;
	gyroZ = ((float)rawGyroZ) / GYRO_SCALE_VALUE;

	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	// Neu dat cum tinh goc nay ben trong doan code been duoi, o giua 2 lan xac dinh interval va preInterval, ro rang se co 1 khoang tgian do giua 2 time bi mat di
	angleAccY = -atan(accX/sqrt(accY*accY + accZ*accZ)) * 180.0 / PI;
	// angleAccX = atan(accY/sqrt(accX*accX + accZ*accZ)) * 180.0 / PI;

	// the same calculation result?
	angleAccX = atan2(accY, accZ + abs(accX)) * 180.0 / PI;
	// angleAccY = atan2(accX, accZ + abs(accY)) * -180.0 / PI;

	// Intergration
	{
		interval = (millis() - preInterval) * 0.001;
		preInterval = millis();
		#ifdef stupid
		angleIntergrate();
		#endif
	}

	// kalman filter
	{
		angleX += gyroX * interval;
		K_uncertainty_roll += interval * interval * 4 * 4;
		float K_gain_roll = K_uncertainty_roll / (K_uncertainty_roll + 3*3);
		K_uncertainty_roll *= (1-K_gain_roll);
		angleX += K_gain_roll * (angleAccX - angleX);

		angleY += gyroY * interval;
		K_uncertainty_pitch += interval * interval * 4 * 4;
		float K_gain_pitch = K_uncertainty_pitch/(K_uncertainty_pitch + 3*3);
		K_uncertainty_pitch *= (1-K_gain_pitch);
		angleY = angleY + K_gain_pitch * (angleAccY-angleY);
	}
}

void MPU6050::test_Kalman_filter(bool y_axis, bool x_axis) {
	long timer = 0;
	while(true) {
		update();
		if(millis() - timer > 5){
			if (y_axis) {
				Serial.print("a : ");	
				Serial.print(angleAccY, 2);
				Serial.print("\tb : ");
				Serial.print(angleGyroY, 2);
				Serial.print("\tc : ");	
				Serial.println(angleY, 2);
				timer = millis();   
			} 
			if (x_axis) {
				Serial.print("d : ");	
				Serial.print(angleAccX, 2);
				Serial.print("\te : ");
				Serial.print(angleGyroX, 2);
				Serial.print("\tf : ");	
				Serial.println(angleX, 2);
				timer = millis();
			}
		}
	}
}
void MPU6050::writeMPU6050(byte reg, byte data){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(reg);
	wire->write(data);
	wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(reg);
	wire->endTransmission(true);
	wire->requestFrom(MPU6050_ADDR, 1);
	byte data =  wire->read();
	return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

void MPU6050::setInterval(float interV) {
	interval = interV;
}

Rate_paras MPU6050::get_gyro_paras() {
	Rate_paras result;
	result.roll = gyroX;
	result.pitch = gyroY;
	result.yaw = gyroZ;
	return result;
}

float MPU6050::getInterval() {
	return interval;
}
