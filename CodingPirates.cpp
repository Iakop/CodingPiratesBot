/*
 ============================================================================
 Source File : CodingPirates.cpp
 Author      : Jacob Bechmann Pedersen
 Date        : 15. mar. 2017
 Version     : 1.0
 Copyright   : Open Source
 Description : Programming exercise
 ============================================================================
*/

#include "CodingPirates.h"

IRsensor::IRsensor(uint8_t pin){
	inPin = pin;
	pinMode(inPin, INPUT);
}

bool IRsensor::read(void){
	return digitalRead(inPin);
}

uint8_t IRsensor::connection(void){
	return inPin;
}

void motorInit(void){
	pinMode(R_PIN, OUTPUT);
	pinMode(R_EN, OUTPUT);
	pinMode(L_PIN, OUTPUT);
	pinMode(L_EN, OUTPUT);
	analogWrite(R_EN, 0);
	analogWrite(L_EN, 0);
}

void forward(uint8_t throttle) {
	digitalWrite(R_PIN, HIGH);
	digitalWrite(L_PIN, HIGH);
	analogWrite(R_EN, throttle);
	analogWrite(L_EN, throttle);
}

void backward(uint8_t throttle) {
	digitalWrite(R_PIN, LOW);
	digitalWrite(L_PIN, LOW);
	analogWrite(R_EN, throttle);
	analogWrite(L_EN, throttle);
}

void turnRight(uint8_t throttle) {
	digitalWrite(R_PIN, LOW);
	digitalWrite(L_PIN, HIGH);
	analogWrite(R_EN, throttle);
	analogWrite(L_EN, throttle);
}

void turnLeft(uint8_t throttle) {
	digitalWrite(R_PIN, HIGH);
	digitalWrite(L_PIN, LOW);
	analogWrite(R_EN, throttle);
	analogWrite(L_EN, throttle);
}

void motorStop(void){
	analogWrite(R_EN, 0);
	analogWrite(L_EN, 0);
}

void motorSpeeds(int left, int right){
	switch(left >= 0){
		case true:
			digitalWrite(L_PIN, HIGH);
			break;
		case false:
			left = left*(-1);
			digitalWrite(L_PIN, LOW);
			break;
		default:
			analogWrite(R_EN, 0);
			analogWrite(L_EN, 0);
			break;
	}
	switch(right >= 0){
		case true:
			digitalWrite(R_PIN, HIGH);
			break;
		case false:
			right = right*(-1);
			digitalWrite(R_PIN, LOW);
			break;
		default:
			analogWrite(R_EN, 0);
			analogWrite(L_EN, 0);
			break;
	}
	analogWrite(R_EN, (uint8_t) right);
	analogWrite(L_EN, (uint8_t) left);
}

void gyroInit(void){
	Wire.begin();
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B);  // PWR_MGMT_1 Register
	Wire.write(0);     // Set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
}

int16_t GyX,GyY,GyZ;

int16_t gyroX(void){
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H)
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDR, 2);  // request a total of 2 registers
	GyX = (Wire.read() <<8| Wire.read());
	return GyX/131;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
}

int16_t gyroY(void){
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x45);  // starting with register 0x45 (GYRO_YOUT_H)
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDR, 2);  // request a total of 2 registers
	GyY = (Wire.read()<<8|Wire.read());
	return GyY/131;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
}

int16_t gyroZ(void){
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x47);  // starting with register 0x47 (GYRO_ZOUT_H)
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDR, 2);  // request a total of 2 registers
	GyZ = (Wire.read()<<8|Wire.read());
	return GyZ;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void turnDegs(signed int deg, uint8_t spd){
	long int traveld = 0;
	long int target = deg*1000*131;
	
	analogWrite(R_EN, spd);
	analogWrite(L_EN, spd);
	unsigned long long curTime = millis();
	switch(deg>=0){
		case true:
			digitalWrite(R_PIN, LOW);
			digitalWrite(L_PIN, HIGH);
			while(traveld < target){
				traveld += gyroZ()*(millis()-curTime);
				curTime = millis();
			}
			break;
		case false:
			digitalWrite(R_PIN, HIGH);
			digitalWrite(L_PIN, LOW);
			while(traveld > target){
				traveld -= 0-(gyroZ()*(millis()-curTime));
				curTime = millis();
			}
			break;
		default:
			analogWrite(R_EN, 0);
			analogWrite(L_EN, 0);
			break;
	}
	analogWrite(R_EN, 0);
	analogWrite(L_EN, 0);
}