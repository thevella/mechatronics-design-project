#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>

extern TwoWire Wire1;

#define E220_30
#define FREQUENCY_915
#define DESTINATION_ADDL 3

#define MOTOR_FR 1
#define MOTOR_FL 2
#define MOTOR_RL 3
#define MOTOR_RR 4

#define MOTOR_FR_FORWARD FORWARD
#define MOTOR_FL_FORWARD BACKWARD
#define MOTOR_RL_FORWARD FORWARD
#define MOTOR_RR_FORWARD BACKWARD

#if MOTOR_FR_FORWARD == FORWARD
#define MOTOR_FR_BACKWARD BACKWARD
const uint8_t MOTOR_FR_ENC_PINS[] = {36, 38};
#else
#define MOTOR_FR_BACKWARD FORWARD
const uint8_t MOTOR_FR_ENC_PINS[] = {38, 36};
#endif

#if MOTOR_FL_FORWARD == FORWARD
#define MOTOR_FL_BACKWARD BACKWARD
const uint8_t MOTOR_FL_ENC_PINS[] = {44, 46};
#else
#define MOTOR_FL_BACKWARD FORWARD
const uint8_t MOTOR_FL_ENC_PINS[] = {46, 44};
#endif

#if MOTOR_RL_FORWARD == FORWARD
#define MOTOR_RL_BACKWARD BACKWARD
const uint8_t MOTOR_RL_ENC_PINS[] = {34, 32};
#else
#define MOTOR_RL_BACKWARD FORWARD
const uint8_t MOTOR_RL_ENC_PINS[] = {32, 34};
#endif

#if MOTOR_RR_FORWARD == FORWARD
#define MOTOR_RR_BACKWARD BACKWARD
const uint8_t MOTOR_RR_ENC_PINS[] = {42, 40};
#else
#define MOTOR_RR_BACKWARD FORWARD
const uint8_t MOTOR_RR_ENC_PINS[] = {40, 42};
#endif

const uint8_t MOTOR_ENC_PINS[][2] = {*MOTOR_FR_ENC_PINS, *MOTOR_FL_ENC_PINS, *MOTOR_RL_ENC_PINS, *MOTOR_RR_ENC_PINS};

#include <LoRa_E220.h>

BLA::ArrayMatrix<2, 1, double> xyc = {0,0};
BLA::ArrayMatrix<2, 1, double> speed_solved = {0, 0};
BLA::ArrayMatrix<2,2, double> speed = {0.70710678118655, -0.70710678118655, 0.70710678118655, 0.70710678118655};
auto speed_decomp = BLA::LUDecompose(speed);

volatile bool do_rotate = false;
bool down = false;

uint16_t max_speed = 4095/2;

uint16_t  deadzone = 100;
uint16_t max_analog = 4095;


// Distance Sensor
uint8_t gpio = 13;
uint8_t vout = A0;

/*
LoRa_E220(HardwareSerial* serial, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
LoRa_E220(HardwareSerial* serial, byte auxPin, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
LoRa_E220(HardwareSerial* serial, byte auxPin, byte m0Pin, byte m1Pin, UART_BPS_RATE bpsRate = UART_BPS_RATE_9600);
*/
// LoRa_E220(serial_interface, digital_pin, digital_pin, digital_pin)
LoRa_E220 e220ttl(&Serial3, 30, 29, 28);


char output[50];

Adafruit_MotorShield motorshield = Adafruit_MotorShield();


struct command {
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

volatile struct encoders enc = {0, 0, 0, 0};
struct command com = {(uint16_t)(max_analog/2), (uint16_t)(max_analog/2), false};
bool old_button = false;


void move(uint16_t, uint16_t);
void rotate(int);

void set_do_rotate() {
    do_rotate = !do_rotate;
}

void MOTOR_FR_ENC_0();
void MOTOR_FR_ENC_1();
void MOTOR_FL_ENC_0();
void MOTOR_FL_ENC_1();
void MOTOR_RL_ENC_0();
void MOTOR_RL_ENC_1();
void MOTOR_RR_ENC_0();
void MOTOR_RR_ENC_1();


void check_message();

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
void lora_setup();


void setup() {
    analogReadResolution(12);
    Serial.begin(9600);           // set up Serial library at 9600 bps
    while (!Serial) {}

    Serial.println("Adafruit Motorshield v2 - DC Motor test!");

    if (!motorshield.begin(1600, &Wire1)) {         // create with the default frequency 1.6KHz
        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    
    Serial.println("Motor Shield found.");


    pinMode(gpio, OUTPUT);
    pinMode(vout, INPUT);
    digitalWrite(gpio, HIGH);
    // Startup all pins and UART

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; i < 2; ++i) {
            pinMode(MOTOR_ENC_PINS[i][j], INPUT);
        }
    }

    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENC_PINS[0]), MOTOR_FR_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENC_PINS[0]), MOTOR_FL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENC_PINS[0]), MOTOR_RL_ENC_0, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENC_PINS[0]), MOTOR_RR_ENC_0, RISING);


    lora_setup();

}

void loop() {
    if (com.button) {
        rotate(com.x);
    } else {
        move(com.x, com.y);
    }

    check_message();
    Serial.println(analogRead(vout));

    delay(5);

    //Serial.println(analogRead(gpio));
    //sprintf(output, "X: %04i, Y: %04i, C: %i", analogRead(x_pin), analogRead(y_pin), do_rotate);
    //Serial.println(output);


}

void move(uint16_t x, uint16_t y) {
    double xc = (max_analog/(double)2.0) - x;
    double yc = (max_analog/(double)2.0) - y;

    if (abs(xc) < deadzone) {
        xc = 0;
    }

    if (abs(yc) < deadzone) {
        yc = 0;
    }

    //double xyc_abs = sqrt(pow(xc/max_analog/2, 2) + pow(yc/max_analog/2, 2));

    xyc = {xc, yc};

    speed_solved = BLA::LUSolve(speed_decomp, xyc);
    
    double speed_abs = sqrt((speed_solved(0)*speed_solved(0)) + (speed_solved(1)*speed_solved(1)));

    double speed_neg = (speed_solved(0)/speed_abs) * max_speed;
    double speed_pos = (speed_solved(1)/speed_abs) * max_speed;


    if (speed_neg > 0) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
    } else {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
    }

    if (speed_pos > 0) {
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else {
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    }
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round(abs(speed_neg*(4822/4702.0))) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round(abs(speed_pos*(4852/4854.0))) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round(abs(speed_neg*(7047/7199.0))) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round(abs(speed_pos)) );
}

void rotate(int x) {
    double xc = (max_analog/2) - x;

    if (abs(xc) < deadzone) {
        xc = 0;
    }
    double speed = (((xc)/(max_analog/2)) * max_speed * (3.0/4)); 

    if (speed < 0) {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_BACKWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_BACKWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_FORWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_FORWARD);
    } else {
        motorshield.getMotor(MOTOR_FL)->run(MOTOR_FL_FORWARD);
        motorshield.getMotor(MOTOR_RL)->run(MOTOR_RL_FORWARD);
        motorshield.getMotor(MOTOR_RR)->run(MOTOR_RR_BACKWARD);
        motorshield.getMotor(MOTOR_FR)->run(MOTOR_FR_BACKWARD);
    }
    

    motorshield.getMotor(MOTOR_FL)->setSpeedFine( (uint16_t)round(abs(speed)) );
    motorshield.getMotor(MOTOR_RL)->setSpeedFine( (uint16_t)round(abs(speed)) );
    motorshield.getMotor(MOTOR_RR)->setSpeedFine( (uint16_t)round(abs(speed)) );
    motorshield.getMotor(MOTOR_FR)->setSpeedFine( (uint16_t)round(abs(speed)) );
}

void check_message() {
    // If something available
    if (e220ttl.available()>1) {
	    // read the String message
#ifdef ENABLE_RSSI
	    ResponseContainer rc = e220ttl.receiveMessageRSSI();
#else
	    ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(command));
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
            
            com.y = data & 0xFFF;
            com.x = ((data >> 12) & 0xFFF);
            com.button = ((data >> 24) & 0x1);
            //sprintf(output, "X: %04i, Y: %04i, C: %i", com.x, com.y, com.button);
            //Serial.println(output);
#ifdef ENABLE_RSSI
            Serial.print("RSSI: "); Serial.println(rc.rssi, DEC);
#endif
        }
    }

    if (com.button != old_button) {
        old_button = com.button;

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

void MOTOR_FR_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FR_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_FR_ENC_PINS[1])) == LOW) {
        enc.FR += 1;
    } else {
        enc.FR -= 1;
    }
}

void MOTOR_FR_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FR_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_FR_ENC_PINS[0])) == LOW) {
        enc.FR += 1;
    } else {
        enc.FR -= 1;
    }
}

void MOTOR_FL_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FL_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_FL_ENC_PINS[1])) == LOW) {
        enc.FL += 1;
    } else {
        enc.FL -= 1;
    }
}

void MOTOR_FL_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_FL_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_FL_ENC_PINS[0])) == LOW) {
        enc.FL += 1;
    } else {
        enc.FL -= 1;
    }
}

void MOTOR_RL_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RL_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_RL_ENC_PINS[1])) == LOW) {
        enc.RL += 1;
    } else {
        enc.RL -= 1;
    }
}

void MOTOR_RL_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RL_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_RL_ENC_PINS[0])) == LOW) {
        enc.RL += 1;
    } else {
        enc.RL -= 1;
    }
}

void MOTOR_RR_ENC_0() {
    //check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RR_ENC_PINS[1])->PIO_PDSR & digitalPinToBitMask(MOTOR_RR_ENC_PINS[1])) == LOW) {
        enc.RR += 1;
    } else {
        enc.RR -= 1;
    }
}

void MOTOR_RR_ENC_1() {//check current state of encoder B's output and return direction
    if ((digitalPinToPort(MOTOR_RR_ENC_PINS[0])->PIO_PDSR & digitalPinToBitMask(MOTOR_RR_ENC_PINS[0])) == LOW) {
        enc.RR += 1;
    } else {
        enc.RR -= 1;
    }
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

