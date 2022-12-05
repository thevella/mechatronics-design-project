#pragma once
#include <Arduino.h>

#include <vl53l4cd_class.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

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


extern int target_deg;

#define DIST_VAL   0
#define DIST_GPIO  1


bool read_TOF_front(int* output, int* status = nullptr);
bool read_TOF_left(int* output, int* status = nullptr);
void test_TOF();

extern bool TEST_FRONT_TOF;
extern bool TEST_LEFT_TOF;

#ifdef USE_IR
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

private:
	uint8_t val_pin;
	uint8_t gpio_pin;
    uint8_t sens_num;
};

extern dist_sensor dist_block;
#endif


extern const uint16_t joystick_deadzone;
extern const uint16_t max_analog;


void setup_sensors();

#ifdef USE_GYRO
double add_degrees(double, double);
bool update_gyro();
double get_rotation();
double deg_difference(double, double);
double get_inclination();
#endif
