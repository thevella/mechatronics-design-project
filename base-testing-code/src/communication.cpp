#include "communication.h"
#include "scheduling.h"
#include "ST25DVSensor.h"

// LORA modules can be used for communication in adverse environments
// due to their low wavelength and simple communications protocol
// Additionally, because of this, LORA is fairly simple to implement
// which is why we are using it for remote control while testing

/*
LoRa_E220(HardwareSerial* serial, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
LoRa_E220(HardwareSerial* serial, byte auxPin, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
LoRa_E220(HardwareSerial* serial, byte auxPin, byte m0Pin, byte m1Pin, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
*/
// LoRa_E220(serial_interface, digital_pin, digital_pin, digital_pin)
LoRa_E220 e220ttl(&Serial3, 30, 29, 28);

// Initialize data we are receiving
struct joystick joystick_com = {(uint16_t)(max_analog/2), (uint16_t)(max_analog/2), false};
bool old_button = false;

char output[50];

EXTERN_COROUTINE(navigate_maze);

String uri;

COROUTINE(nfc_read_call) {
	COROUTINE_LOOP() {
		st25dv.readURI(&uri);

		if (strcmp(uri.c_str(), STR_RB_START_STOP) == 0) {
			if (!navigate_maze.isSuspended()) {
				navigate_maze.suspend();
			} else {
				navigate_maze.resume();
			}
			st25dv.writeURI("", "NULL", "");
		} else {
			;
		}
		COROUTINE_DELAY(50);
	}
	
}



void nfc_setup() {
	st25dv.begin(A1, A2, &Wire);
}



// Check if message available and convert data from struct to 
// usable structure
void check_message_lora() {
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
            int data = *(int*) rc.data;
            // Manually extract data since architecture and compiler are different
			// and since we are using bit fields, on the due side the order is different, 
			// so the data cannot just be cast.

			// This is a workaround since it doesn't need to scale for arbitrary data
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

	// Send debugging data if button on joystick has been pressed

	// Can only send or recevie at any given time, so must be careful not to be 
	// doing anything else while pressing the button

	// Currently up to human operator
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
}

// Initialize lora module
void lora_setup() {
    e220ttl.begin();

	ResponseStructContainer c;
	c = e220ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParametersLora(configuration);


//	----------------------- DEFAULT TRANSPARENT -----------------------
	// Change address so can communicate with other module
	configuration.ADDL = 0x02;  // First part of address
	configuration.ADDH = 0x00; // Second part

	// Channel sets the offset from the base frequency.
	// EG if set to 900hz and channel 23, transmitting at 923hz
	configuration.CHAN = 23; // Communication channel

	configuration.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate

	// Set fast baud rate
	configuration.SPED.airDataRate = AIR_DATA_RATE_101_192; // Air baud rate
	configuration.SPED.uartParity = MODE_00_8N1; // Parity bit

	// Set small packet size, so speed is increased and controls feel realtime
	configuration.OPTION.subPacketSetting = SPS_064_10; // Packet size
	configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special command

	// Increase device power so objects in view do not interrupt
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

	printParametersLora(configuration);
	c.close();
}

// Print current config
void printParametersLora(struct Configuration configuration) {
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