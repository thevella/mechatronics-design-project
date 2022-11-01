#pragma once
#include <Arduino.h>

#include "options.h"

#include "sensors.h"

// Set type of module and frequency,
// must operate in the 915 range to be legal in canada
#define E220_30
#define FREQUENCY_915

// Set the address of the remote
#define DESTINATION_ADDL 3
#include <LoRa_E220.h>

#define STR_RB_START_STOP "0"

extern struct joystick joystick_com;


void nfc_setup();
void check_message_lora();

void printParametersLora(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
void lora_setup();