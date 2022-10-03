#include "main.h"
#include "movement.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>

bool do_rotate = false;

void center(CENTER_TYPE dir) {
    if (dir == CENTER_F_B) {

    } else if (dir == CENTER_L_R) {

    }
}


void setup() {
    analogReadResolution(12);
    Serial.begin(9600);           // set up Serial library at 9600 bps
    while (!Serial) {}

    setup_sensors();
    setup_movement();

    lora_setup();

}

void loop() {
    if (com.button) {
        manual_rotate(com.x);
    } else {
        manual_move(com.x, com.y);
    }

    check_message();
    Serial.println(analogRead(distance_sensors[DIST_LEFT][DIST_VAL]));

    delay(5);

    //Serial.println(analogRead(gpio));
    //sprintf(output, "X: %04i, Y: %04i, C: %i", analogRead(x_pin), analogRead(y_pin), do_rotate);
    //Serial.println(output);
}



void set_do_rotate() {
    do_rotate = !do_rotate;
}



