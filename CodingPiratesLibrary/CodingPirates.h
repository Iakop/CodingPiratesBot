/*
 ============================================================================
 Source File : CodingPirates.h
 Author      : Jacob Bechmann Pedersen
 Date        : 15. mar. 2017
 Version     : 1.0
 Copyright   : Open Source
 Description : Programming exercise
 ============================================================================
*/
#ifndef CODINGPIRATES_H_
#define CODINGPIRATES_H_

#include "Arduino.h"
#include <Wire.h>

#define R_PIN	4
#define R_EN 	6
#define L_PIN	7
#define L_EN 	5

#define BR 14
#define FR 15
#define BL 16
#define FL 17

#define U1_ECHO 8
#define U1_TRIG 9
#define U2_ECHO 10
#define U2_TRIG 11
#define U3_ECHO 12
#define U3_TRIG 13

#define MPU_ADDR 0x68

class IRsensor {
	public:
		IRsensor(uint8_t pin);
		bool read(void);
		uint8_t connection(void);
	private:
		uint8_t inPin;
};

void motorInit(void);
void gyroInit(void);

void forward(uint8_t throttle);
void backward(uint8_t throttle);
void turnRight(uint8_t throttle);
void turnLeft(uint8_t throttle);
void motorStop(void);
void motorSpeeds(int left, int right);

int16_t gyroX(void);
int16_t gyroY(void);
int16_t gyroZ(void);

void turnDegs(signed int deg, uint8_t spd);

#endif /* CODINGPIRATES_H_ */
