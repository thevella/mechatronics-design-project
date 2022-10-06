#include "communication.h"

/*
LoRa_E220(HardwareSerial* serial, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
LoRa_E220(HardwareSerial* serial, byte auxPin, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
LoRa_E220(HardwareSerial* serial, byte auxPin, byte m0Pin, byte m1Pin, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
*/
// LoRa_E220(serial_interface, digital_pin, digital_pin, digital_pin)
LoRa_E220 e220ttl(&Serial3, 30, 29, 28);

struct joystick joystick_com = {(uint16_t)(max_analog/2), (uint16_t)(max_analog/2), false};
bool old_button = false;

char output[50];


void check_message() {
    // If something available
    if (e220ttl.available()>1) {
	    // read the String message
#ifdef ENABLE_RSSI
	    ResponseContainer rc = e220ttl.receiveMessageRSSI();
#else
	    ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(joystick));
#endif
        // Is something goes wrong print error
        if (rc.status.code!=1){
            Serial.println("error");
            Serial.println(rc.status.getResponseDescription());
        }else{
            // Print the data received
            //Serial.println(rc.status.getResponseDescription());
            //Serial.println("we got it");
            //Serial.println(*(int*) rc.data);
            int data = *(int*) rc.data;
            
            joystick_com.y = data & 0xFFF;
            joystick_com.x = ((data >> 12) & 0xFFF);
            joystick_com.button = ((data >> 24) & 0x1);
            //sprintf(output, "X: %04i, Y: %04i, C: %i", com.x, com.y, com.button);
            //Serial.println(output);
#ifdef ENABLE_RSSI
            Serial.print("RSSI: "); Serial.println(rc.rssi, DEC);
#endif
        }
    }


    if (joystick_com.button != old_button) {
        old_button = joystick_com.button;

        #ifdef USE_ENCODERS
        noInterrupts();
        struct encoders enc_temp = {enc.FR, enc.FL, enc.RL,enc.RR};
        enc.FR = 0;
        enc.FL = 0;
        enc.RL = 0;
        enc.RR = 0;
        interrupts();

        ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 23, &enc_temp, sizeof(encoders));
        sprintf(output, "FL: %05li, FR: %05li\nRL: %05li, RR: %05li\n", enc_temp.FL, enc_temp.FR, enc_temp.RL, enc_temp.RR);
        Serial.println(output);
        #endif
    }

    /*
    if (Serial.available()) {
        String input = Serial.readString();
        ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 23, input);
        // Check If there is some problem of succesfully send
        Serial.print("Sending: ");
        Serial.println(input);
        Serial.println(rs.getResponseDescription());
    }
    */
}

void lora_setup() {
    e220ttl.begin();

	ResponseStructContainer c;
	c = e220ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParameters(configuration);

//	----------------------- DEFAULT TRANSPARENT -----------------------
	configuration.ADDL = 0x02;  // First part of address
	configuration.ADDH = 0x00; // Second part

	configuration.CHAN = 23; // Communication channel

	configuration.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
	configuration.SPED.airDataRate = AIR_DATA_RATE_101_192; // Air baud rate
	configuration.SPED.uartParity = MODE_00_8N1; // Parity bit

	configuration.OPTION.subPacketSetting = SPS_064_10; // Packet size
	configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special command
	configuration.OPTION.transmissionPower = POWER_30; // Device power

	configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED; // Enable RSSI info
	configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION; // Enable repeater mode
	configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED; // Check interference
	configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011; // WOR timing

    // Set configuration changed and set to not hold the configuration
	ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
	Serial.println(rs.getResponseDescription());
	Serial.println(rs.code);

	c = e220ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParameters(configuration);
	c.close();
}


void printParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
	Serial.println(F(" "));
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
	Serial.println(F(" "));
	Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
	Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
	Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
	Serial.println(F(" "));
	Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
	Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
	Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
	Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

	Serial.println("----------------------------------------");
}