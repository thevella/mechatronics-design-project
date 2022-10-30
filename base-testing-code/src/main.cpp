
#include "main.h"
#include "movement.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>



bool do_rotate = false;


void setup() {
    // Small delay so we can clear the wires before it starts
    delay(1000);
    // Increase resolution so less error in reading sensors
    // new range 0-4095
    analogReadResolution(12);

    // Start serial interface for debugging
    Serial.begin(9600);
    while (!Serial) {}

    // Call setup functions
    setup_sensors();
    setup_movement();


    int rot = get_rotation();

    while(true) {
        manual_move(4096/2-1000,4096/2+1000);
        delay(5000);
        while(abs(get_rotation() + 110) > 2) {
            robot_rotation(RB_TURN_CW);
            delay(100);
        }
        robot_move(RB_STOP);

        manual_move(4096/2-1000,4096/2+1000);
        delay(500);
        robot_move(RB_STOP);

        Serial.println(get_rotation());
    }
    // Only pull in lora if we want to use it, otherwise
    // will complain and cause slowdowns if unplugged
    #ifdef USE_LORA
    lora_setup();
    #endif
}



void loop() {
    // How far off the wall as read by the sensor to stop
    int sensor_offset = 2570;

    // Begin movement
    recenter();

    // Stay further from the wall,
    // avoid hitting the posts
    forward_sense(sensor_offset+5);

    turn(RB_TURN_CW);

    // Correction strafe
    strafe(RB_LEFT, -2200);

    // Correct angle off wall
    backward();

    // Move forward one square and center
    forward(1, 150, true, true);

    forward_sense(sensor_offset);

    // Increase strafing slightly to adjust for
    // clearing the post
    strafe(RB_RIGHT, 50);

    backward();

    forward_sense(sensor_offset);

    strafe(RB_LEFT, 75);

    backward();

    // Ramp

    // Go close to full speed since not worried about drift
    // on the ramp
    forward_sense(sensor_offset, max_speed*1.7);

    turn(RB_TURN_CW);

    backward();

    forward_sense(sensor_offset, max_speed*1.7);

    turn(RB_TURN_CW);

    forward_sense(sensor_offset, max_speed*1.7);

    turn(RB_TURN_CW);

    forward(2, 600, true, true, false);

    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    backward();

    // Correction strafe
    strafe(RB_LEFT, -2000);

    forward_sense(sensor_offset);

    // Bottom

    turn(RB_TURN_CW, 200);

    backward();

    forward(1, 150, true, true);

    forward_sense(sensor_offset+5);

    turn(RB_TURN_CC);

    backward();

    // Long hallway, have to correct twice
    forward(1, 150, true, true);

    forward(2, 100, true, true);

    forward_sense(sensor_offset);

    turn(RB_TURN_CC, 50);

    strafe(RB_RIGHT, -2150);

    backward();

    forward(1, 150);

    turn(RB_TURN_CC);

    strafe(RB_RIGHT, -2250);

    backward();

    forward(1, 150, true, true);

    forward_sense(sensor_offset);

    turn(RB_TURN_CW);

    backward();

    forward_sense(sensor_offset+75);

    turn(RB_TURN_CC);

    strafe(RB_RIGHT, -2100);

    backward();

    forward(1, 150, true, true);

    // Grab soil
    grip_sand();

    backward();

    // End of sequence, idle
    while(true) {}

}









