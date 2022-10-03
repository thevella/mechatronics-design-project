#pragma once
#include <Arduino.h>
#include "sensors.h"

#define E220_30
#define FREQUENCY_915
#define DESTINATION_ADDL 3
#include <LoRa_E220.h>

extern struct joystick joystick_com;

void check_message();

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
void lora_setup();