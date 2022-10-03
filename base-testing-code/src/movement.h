#pragma once
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
#include "communication.h"
#include "sensors.h"
#include "movement_const.h"

enum CENTER_TYPE {
    CENTER_F_B,
    CENTER_L_R
};

enum ROBOT_DIR {
    RB_FORWARD,
    RB_BACKWARD,
    RB_RIGHT,
    RB_LEFT,
    RB_TURN_CC,
    RB_TURN_CW
};

void manual_move(uint16_t, uint16_t);
void manual_rotate(int);



void setup_movement();