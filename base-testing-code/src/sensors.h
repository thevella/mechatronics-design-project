#pragma once
#include <Arduino.h>

#include <SparkFunMPU9250-DMP.h>

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

#define DIST_VAL   0
#define DIST_GPIO  1




class dist_sensor {
	
public:
	dist_sensor(uint8_t sensor);

	uint16_t raw_value() {return analogRead(this->val_pin);};
	uint16_t read_dist() {
		return this->a / (this->raw_value() - this->b);
	};

	void change_state(bool turn_on) {
		if (turn_on) {
			digitalWrite(this->gpio_pin, HIGH);
		} else {
			digitalWrite(this->gpio_pin, LOW);
		}
	};

	void calibrate(uint16_t val1, uint16_t val2);

private:
	uint16_t cal_distances_mm[2];
	uint16_t cal_values[2];
	uint32_t a;
	uint32_t b;
	uint8_t val_pin;
	uint8_t gpio_pin;
};

extern dist_sensor dist_left;
extern dist_sensor dist_right;
extern dist_sensor dist_front;
extern dist_sensor dist_back;



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


float add_degrees(float, float);
bool update_gyro();
float get_rotation();
