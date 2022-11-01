#pragma once
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
#include "communication.h"
#include "sensors.h"
#include "movement_const.h"
#include <Servo.h>

#include "scheduling.h"

extern uint16_t robot_acceleration;
extern uint16_t max_speed;

void robot_move(ROBOT_DIR, uint16_t speed = max_speed, uint16_t acceleration = robot_acceleration);

void robot_rotation(ROBOT_DIR, uint16_t speed = max_speed, uint16_t acceleration = robot_acceleration);

void robot_rotation_by_deg (ROBOT_DIR, uint16_t, uint16_t = max_speed, uint16_t = robot_acceleration);

void manual_move(uint16_t, uint16_t);
void manual_rotate(int);

void setup_movement();

void recenter();

void grip_sand();

void forward(int , int16_t delay_offset = 0, bool center = false, bool hard = false, bool on_right = true);

void forward_sense(int , int speed = max_speed);

void backward(int time_offset = 0);

void strafe(ROBOT_DIR , int time_offset = 0);


void turn(ROBOT_DIR , int time_offset = 0);

