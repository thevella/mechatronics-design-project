
#include "main.h"
#include "movement.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>

bool do_rotate = false;

#ifdef USE_SCHEDULING
using namespace ace_routine;
#endif

void setup() {
    // Small delay so we can clear the wires before it starts
    delay(1000);
    // Increase resolution so less error in reading sensors
    // new range 0-4095
    analogReadResolution(12);

    // Start serial interface for debugging
    Serial.begin(9600);
    while (!Serial) {}

    #ifdef USE_NFC
    nfc_setup();
    #endif
    
    // Call setup functions
    setup_sensors();
    setup_movement();



    // Only pull in lora if we want to use it, otherwise
    // will complain and cause slowdowns if unplugged
    #ifdef USE_LORA
    lora_setup();
    #endif



    
}



void loop() {

    navigate_maze();

    while(true){}

   
}









