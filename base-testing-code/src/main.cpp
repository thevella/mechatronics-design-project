
#include "main.h"
#include "movement.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>


bool do_rotate = false;


void setup() {
    delay(1000);
    analogReadResolution(12);

    Serial.begin(9600);           // set up Serial library at 9600 bps
    while (!Serial) {}

    setup_sensors();
    setup_movement();

    //lora_setup();



}

void forward(int number, int16_t delay_offset = 0, bool center = false, bool hard = false) {
    const int move_delay = 2050;


    if (number >= 2) {
        for (int i = 0; i < number/2; ++i) {
            robot_move(RB_FORWARD);
            delay((move_delay+delay_offset)*2);
            robot_move(RB_STOP);
            delay(500);
            if (center) {
                if (hard) {
                    robot_move(RB_RIGHT);
                    delay(1000);
                    robot_move(RB_STOP);
                    delay(500);
                }
                recenter();
                delay(500);
            } 
        }
    }

    if (number % 2 == 1) {
        robot_move(RB_FORWARD);
        delay(move_delay + delay_offset);
        robot_move(RB_STOP);
        delay(500);
    }

    if (center) {
        if (hard) {
            robot_move(RB_RIGHT);
            delay(1000);
            robot_move(RB_STOP);
            delay(500);
        }
        recenter();
        delay(500);
    }
    

}

void forward_sense(int offset) {
    robot_move(RB_FORWARD);

    //2410 54
    //1954 120

    while (dist_front.raw_value() < offset) {
        delay(10);
    }

    robot_move(RB_STOP);

    delay(500);

}

void backward(int time_offset = 0) {
    robot_move(RB_BACKWARD);

    delay(700 + time_offset);

    robot_move(RB_STOP);

    delay(500);
}

void strafe(ROBOT_DIR dir, int time_offset = 0) {
    robot_move(dir);

    delay(2300 + time_offset);

    robot_move(RB_STOP);

    delay(500);
}

void turn(ROBOT_DIR dir, int time_offset = 0) {
    robot_rotation(dir);

    delay(1800 + time_offset);

    robot_move(RB_STOP);

    delay(500);
}

void loop() {
    int sensor_offset = 2570;
    // if (joystick_com.button) {
    //     manual_rotate(joystick_com.x);
    // } else {
    //     manual_move(joystick_com.x, joystick_com.y);
    // }

    //check_message();
    //Serial.println(dist_left.raw_value());

    //delay(5);

    //Serial.println(analogRead(gpio));
    //sprintf(output, "X: %04i, Y: %04i, C: %i", analogRead(x_pin), analogRead(y_pin), do_rotate);
    //Serial.println(output);

    recenter();

    //forward(1, 100, false);
    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    backward();

    forward(1, 0, true, true);

    forward_sense(sensor_offset);

    strafe(RB_RIGHT);

    backward();

    forward_sense(sensor_offset);

    strafe(RB_LEFT);

    backward();

    // Ramp

    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    backward();

    //forward(1, 0, true, true);

    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    //forward(1, 200, true, true);

    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    //forward(1, 500, true);

    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    backward();

    forward_sense(sensor_offset);

    

    // Bottom

    turn(RB_TURN_CW);

    backward();

    forward_sense(sensor_offset);

    turn(RB_TURN_CC);

    backward();

    forward(1, 600, true, true);

    forward_sense(sensor_offset);

    turn(RB_TURN_CC);

    backward();

    forward(1, 500);

    turn(RB_TURN_CC);

    backward();

    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    backward();

    forward_sense(sensor_offset);

    turn(RB_TURN_CC);

    backward();

    forward(1, 500, true, true);

    // Grab soil

    


    while(true) {}

}









