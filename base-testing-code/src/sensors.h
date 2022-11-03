#pragma once
#include <Arduino.h>

#include <SparkFunMPU9250-DMP.h>
#include <vl53l4cd_class.h>

#include "options.h"


/**
 * @brief   Joystick object for standard usage, thus can change if local
 *          or remote control.
 * 
 */
struct joystick {
	uint16_t y : 12;
	uint16_t x : 12;
	boolean button : 1; 
};

#ifdef USE_ENCODERS
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
#endif

#define DIST_VAL   0
#define DIST_GPIO  1


extern int target_deg;
bool read_TOF_front(int* output, int* status = nullptr);
bool read_TOF_left(int* output, int* status = nullptr);


extern bool TEST_FRONT_TOF;
extern bool TEST_LEFT_TOF;


class dist_sensor {
	
public:
	dist_sensor(uint8_t sensor);

    // 
    /**
     * @brief           Take an average of the sensor readings over 100ms
     * 
     * @return uint16_t Averaged sensor reading
     * 
     * @details         Average readings with small delay to get better
     *                  results and not as much fluctuation
     * 
     *                  Reading too fast can result in either garbage data, or the same
     *                  value multiple times
     */
	uint16_t raw_value() {
        uint64_t temp = 0;

        for (int i = 0; i < 10; ++i) {
            temp += analogRead(this->val_pin);
            delay(10);
        }

        return temp/10;
    };

    /**
     * @brief       Linearly interpolate distance from calibration data
     * 
     * @return int  Distance from sensor to object (mm)
     */
	int read_dist() {
        uint16_t temp = this->raw_value();
        
        int temp2 = round(this->cal_distances_mm[0]+(((temp-this->cal_values[0])/((double)(this->cal_values[1] - this->cal_values[0])))*(this->cal_distances_mm[1] - this->cal_distances_mm[0])));


        #ifdef DEBUG_PRINT_DIST
        char out1[50] = "\0";
        
        sprintf(out1, "RAW %i: %i", this->sens_num, temp);
        
        Serial.println(out1);
        
        sprintf(out1, "RAW_DIST BEFORE %i: %i", this->sens_num, temp2);
        
        Serial.println(out1);
        #endif

        return temp2;
	};

    /**
     * @brief       Use offsets to ignore the sensors distance into the frame, so that each sensor
     *              reads as if it starts at the edge of the frame
     * 
     * @return int  Distance from side of robot to object (mm)
     */
    int read_dist_wheels() {
        int temp1 = this->read_dist();

        #ifdef DEBUG_PRINT_DIST
        char out1[50] = "\0";
        sprintf(out1, "RAW_DIST %i: %i", this->sens_num, temp1);
        Serial.println(out1);
        #endif

        int temp = temp1 - this->cal_distances_mm[0];
        
        #ifdef DEBUG_PRINT_DIST
        sprintf(out1, "DIST %i: %i", this->sens_num, temp);
        Serial.println(out1);
        //delay(2000);
        #endif

        return temp;
    };

    /**
     * @brief           Turn the sensor on or off, can be used
     *                  to conserve power
     * 
     * @param turn_on   True if turning on, else turn off
     */
	void change_state(bool turn_on) {
		if (turn_on) {
			digitalWrite(this->gpio_pin, HIGH);
		} else {
			digitalWrite(this->gpio_pin, LOW);
		}
	};

	void calibrate(uint16_t val1, uint16_t val2);

private:
	uint32_t cal_distances_mm[2];
	uint16_t cal_values[2];
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

#ifdef USE_ENCODERS
void MOTOR_FR_ENC_0();
void MOTOR_FR_ENC_1();
void MOTOR_FL_ENC_0();
void MOTOR_FL_ENC_1();
void MOTOR_RL_ENC_0();
void MOTOR_RL_ENC_1();
void MOTOR_RR_ENC_0();
void MOTOR_RR_ENC_1();
#endif

void setup_sensors();

#ifdef USE_GYRO
float add_degrees(float, float);
bool update_gyro();
float get_rotation();
float deg_difference(float, float);
#endif
