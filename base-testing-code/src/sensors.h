#pragma once
#include <Arduino.h>

struct joystick {
	uint16_t y : 12;
	uint16_t x : 12;
	boolean button : 1; 
};

struct encoders {
	int32_t FR;
	int32_t FL;
	int32_t RL;
	int32_t RR;
};

extern const uint8_t MOTOR_ENC_PINS[][2];
extern const uint8_t MOTOR_FR_ENC_PINS[];
extern const uint8_t MOTOR_FL_ENC_PINS[];
extern const uint8_t MOTOR_RL_ENC_PINS[];
extern const uint8_t MOTOR_RR_ENC_PINS[];
extern volatile struct encoders enc;


#define DIST_LEFT  0
#define DIST_RIGHT 1
#define DIST_FRONT 2
#define DIST_BACK  3

#define DIST_VAL   0
#define DIST_GPIO  1

extern const uint8_t distance_sensors[][2];
extern const uint8_t num_dist_sensors;
extern const uint16_t joystick_deadzone;
extern const uint16_t max_analog;

void MOTOR_FR_ENC_0();
void MOTOR_FR_ENC_1();
void MOTOR_FL_ENC_0();
void MOTOR_FL_ENC_1();
void MOTOR_RL_ENC_0();
void MOTOR_RL_ENC_1();
void MOTOR_RR_ENC_0();
void MOTOR_RR_ENC_1();

void setup_sensors();
