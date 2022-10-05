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

	uint16_t raw_value() {
        // char out1[50] = "\0";
        // sprintf(out1, "RAW %i: %i", this->sens_num, analogRead(this->val_pin));
        // Serial.println(out1); 
        return analogRead(this->val_pin);
    };
	int64_t read_dist() {
        uint16_t temp = this->raw_value();
        return (this->cal_distances_10nm[0]*(this->cal_values[0] - temp)+this->cal_distances_10nm[1]*(temp - this->cal_values[1]))/((double)(this->cal_values[1] - this->cal_values[0]));
		//return this->a / (this->raw_value() - this->b);
	};

    int64_t read_dist_wheels() {
        int32_t temp = -this->read_dist() - this->cal_distances_10nm[0];
        char out1[50] = "\0";
        sprintf(out1, "RAW %li: %li", this->sens_num, temp);
        Serial.println(out1);
        return temp;
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
	uint32_t cal_distances_10nm[2];
	uint16_t cal_values[2];
	uint64_t a;
	uint64_t b;
	uint8_t val_pin;
	uint8_t gpio_pin;
    uint8_t sens_num;
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
