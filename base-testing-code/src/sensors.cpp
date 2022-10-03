#include "sensors.h"

#if MOTOR_FR_FORWARD == FORWARD
const uint8_t MOTOR_FR_ENC_PINS[] = {36, 38};
#else
const uint8_t MOTOR_FR_ENC_PINS[] = {38, 36};
#endif

#if MOTOR_FL_FORWARD == FORWARD
const uint8_t MOTOR_FL_ENC_PINS[] = {44, 46};
#else
const uint8_t MOTOR_FL_ENC_PINS[] = {46, 44};
#endif

#if MOTOR_RL_FORWARD == FORWARD
const uint8_t MOTOR_RL_ENC_PINS[] = {34, 32};
#else
const uint8_t MOTOR_RL_ENC_PINS[] = {32, 34};
#endif

#if MOTOR_RR_FORWARD == FORWARD
const uint8_t MOTOR_RR_ENC_PINS[] = {42, 40};
#else
const uint8_t MOTOR_RR_ENC_PINS[] = {40, 42};
#endif

const uint8_t MOTOR_ENC_PINS[4][2] = {*MOTOR_FR_ENC_PINS, *MOTOR_FL_ENC_PINS, *MOTOR_RL_ENC_PINS, *MOTOR_RR_ENC_PINS};
volatile struct encoders enc = {0, 0, 0, 0};

// Distance Sensors
const uint8_t distance_sensors[][2] = {{A0, 13}, {A1, 14}};
const uint8_t num_dist_sensors = 2;

const uint16_t joystick_deadzone = 100;
const uint16_t max_analog = 4095;

void setup_sensors() {
    for (int i = 0; i < num_dist_sensors; ++i) {
        pinMode(distance_sensors[i][DIST_VAL], INPUT);
        pinMode(distance_sensors[i][DIST_GPIO], OUTPUT);
        digitalWrite(distance_sensors[i][DIST_GPIO], HIGH);
    }
    // Startup all pins and UART

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; i < 2; ++i) {
            pinMode(MOTOR_ENC_PINS[i][j], INPUT);
        }
    }

    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_PINS[0]), MOTOR_FR_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_PINS[0]), MOTOR_FL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_PINS[0]), MOTOR_RL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_PINS[0]), MOTOR_RR_ENC_0, RISING);

}

void MOTOR_FR_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FR_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_FR_ENC_PINS[1])) == LOW) {
        enc.FR += 1;
    } else {
        enc.FR -= 1;
    }
}

void MOTOR_FR_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FR_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_FR_ENC_PINS[0])) == LOW) {
        enc.FR += 1;
    } else {
        enc.FR -= 1;
    }
}

void MOTOR_FL_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FL_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_FL_ENC_PINS[1])) == LOW) {
        enc.FL += 1;
    } else {
        enc.FL -= 1;
    }
}

void MOTOR_FL_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FL_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_FL_ENC_PINS[0])) == LOW) {
        enc.FL += 1;
    } else {
        enc.FL -= 1;
    }
}

void MOTOR_RL_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RL_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_RL_ENC_PINS[1])) == LOW) {
        enc.RL += 1;
    } else {
        enc.RL -= 1;
    }
}

void MOTOR_RL_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RL_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_RL_ENC_PINS[0])) == LOW) {
        enc.RL += 1;
    } else {
        enc.RL -= 1;
    }
}

void MOTOR_RR_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RR_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_RR_ENC_PINS[1])) == LOW) {
        enc.RR += 1;
    } else {
        enc.RR -= 1;
    }
}

void MOTOR_RR_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RR_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_RR_ENC_PINS[0])) == LOW) {
        enc.RR += 1;
    } else {
        enc.RR -= 1;
    }
}