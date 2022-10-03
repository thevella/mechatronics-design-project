#include <Arduino.h>

#define E220_30
#define FREQUENCY_915
#define LoRa_E220_DEBUG
#include <LoRa_E220.h>
#include <SoftwareSerial.h>
#include <BitBool.h>

#define DESTINATION_ADDL 2

// ---------- Arduino pins --------------
// LoRa_E220(byte txE220pin, byte rxE220pin, byte auxPin, byte m0Pin, byte m1Pin, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
// GRND, VCC, AUX, TX, RX, M1, M0
SoftwareSerial myserial(2,3);
LoRa_E220 e220ttl(&myserial, 5, 7, 6); // Arduino RX <-- e220 TX, Arduino TX --> e220 RX AUX M0 M1


uint8_t x_pin = A0;
uint8_t y_pin = A1;
uint8_t button_pin = 8;
bool button_down = false;

bool rotate = false;


char output[90];

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);

struct joystick {
	uint16_t y : 12;
	uint16_t x : 12;
	boolean button : 1; 
};

struct encoders {
	int32_t FR;
	int32_t FL;
	int32_t RL;
	int32_t RR;
};

struct encoders enc = {0,0,0,0};

struct joystick old_com = {10,15, true};
struct joystick joystick_com = {10, 15, true};


void setup() {
	Serial.begin(9600);
	delay(500);

	pinMode(x_pin, INPUT);
	pinMode(y_pin, INPUT);
	pinMode(button_pin, INPUT);
	digitalWrite(button_pin, HIGH);

	e220ttl.begin();

	ResponseStructContainer c;
	c = e220ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParameters(configuration);

//	----------------------- DEFAULT TRANSPARENT -----------------------
	configuration.ADDL = 0x03;  // First part of address
	configuration.ADDH = 0x00; // Second part

	configuration.CHAN = 23; // Communication channel

	configuration.SPED.uartBaudRate = UART_BPS_9600; // Serial baud rate
	configuration.SPED.airDataRate = AIR_DATA_RATE_101_192; // Air baud rate
	configuration.SPED.uartParity = MODE_00_8N1; // Parity bit

	configuration.OPTION.subPacketSetting = SPS_064_10; // Packet size
	configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED; // Need to send special joystick
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



void loop() {
	// If something available
    if (e220ttl.available()>1) {
	    // read the String message
#ifdef ENABLE_RSSI
	    ResponseContainer rc = e220ttl.receiveMessageRSSI();
#else	
	    ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(encoders));
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

			/*
            uint64_t *data = (uint64_t*) rc.data;
            
            enc.FR = data[0] & 0xFFFFFFFFFFFFFFFF;
            enc.FL = (data[1]) & 0xFFFFFFFFFFFFFFFF;
			enc.RL = (data[2]) & 0xFFFFFFFFFFFFFFFF;
			enc.RR = (data[3]) & 0xFFFFFFFFFFFFFFFF;
			*/
			enc = *(encoders*) rc.data;
            sprintf(output, "FL: %08li, FR: %08li", enc.FL, enc.FR);
            Serial.println(output);
			sprintf(output, "RL: %08li, RR: %08li", enc.RL, enc.RR);
            Serial.println(output);
#ifdef ENABLE_RSSI
            Serial.print("RSSI: "); Serial.println(rc.rssi, DEC);
#endif
        }
    }

	/*
	if (Serial.available()) {
			String input = Serial.readString();
			ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 23, input);
			// Check If there is some problem of succesfully send
			Serial.print("Sending: ");
			Serial.println(input);
			Serial.println(rs.getResponseDescription());
	}*/

	if (digitalRead(button_pin) == LOW && !button_down) {
		button_down = true;
		rotate = !rotate;
	} else if (digitalRead(button_pin) == HIGH) {
		button_down = false;
	}
	// sprintf(output, "RAW -- X: %04i, Y: %04i, C: %i", analogRead(x_pin), analogRead(y_pin), rotate);
	// Serial.println(output);
	uint16_t x = (uint16_t)round((analogRead(x_pin)/(double)1023.0) * 4095);
	uint16_t y = (uint16_t)round((analogRead(y_pin)/(double)1023.0) * 4095);

	if (abs((4095/2.0) - x) < abs((4095/2.0) - y)) {
		x = (4095/2);
	} else {
		y = (4095/2);
	}

	joystick_com = {x, y, rotate};

	if (abs(joystick_com.y - old_com.y) > 15 || abs(joystick_com.x - old_com.x) > 15 || joystick_com.button != old_com.button) {
		//sprintf(output, "X: %04i, Y: %04i, C: %i\n", com.x, com.y, com.button);
		//Serial.println(output);
		ResponseStatus rs = e220ttl.sendFixedMessage(0, DESTINATION_ADDL, 23, &joystick_com, sizeof(joystick));
		old_com = joystick_com;
	}
	//Serial.println((int)sizeof(joystick));

	delay(10);
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