#include "sensors.h"

#ifdef USE_SCHEDULING
#include <AceRoutine.h>
#endif

#define reverse_sensor

// The pins used by the sensors in each location
#define DIST_LEFT  0
#define DIST_RIGHT 1
#define DIST_FRONT 2
#define DIST_BACK  3


#ifdef USE_IR
// Distance Sensors definition
const uint8_t distance_sensors[][2] = {{A0, 13}, {A1, 13}, {A2, 13}, {A3, 13}};
const uint8_t num_dist_sensors = 3;
// Holder for calibration values, if any defined
uint16_t distance_sensor_cal_values[][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};

// Offsets for calculating distance to wall from edge of robot
uint16_t robot_width_mm = 155;
uint16_t wall_to_wall_width_mm = 175+10*2-3;

// distances in mm for calibration
uint64_t distance_mm_vals[][2] = {{(43)*1, (wall_to_wall_width_mm-robot_width_mm+43)*1}, 
                                    {(45)*1, (wall_to_wall_width_mm-robot_width_mm+45)*1}, 
                                    {(54)*1, (120+54)*1}};

// Initialize sensors
dist_sensor dist_left(DIST_LEFT);
dist_sensor dist_right(DIST_RIGHT);
dist_sensor dist_front(DIST_FRONT);
// dist_sensor dist_back(DIST_BACK);
#endif

VL53L4CD TOF_left(&Wire, 52);
VL53L4CD TOF_front(&Wire, 53);

// Ignore small section at middle of joystick where readings
// are often fluctuating
const uint16_t joystick_deadzone = 100;
const uint16_t max_analog = 4095;

#ifdef USE_GYRO
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#endif

bool TEST_FRONT_TOF = false;
bool TEST_LEFT_TOF = false;

int target_deg = 0;


/**
 * @brief Setup the sensors for use, ensure they are ready before continuing
 * 
 */
void setup_sensors() {


    // If using gyro, ensure that we are connected to it, and it is ready
    #ifdef USE_GYRO
    
    if (!bno.begin()) {
        Serial.print("No BNO055 detected");
        while (1);
    }
    
    do {
        uint8_t cals[4] = {0};
        do {
            bno.getCalibration(&(cals[0]), &(cals[1]), &(cals[2]), &(cals[3]));
            delay(50);
        } while (cals[0] != 3 && cals[1] != 3 && cals[2] != 3 && cals[3] != 3);
    } while(false);

    bno.setExtCrystalUse(true);

    bno.setMode(OPERATION_MODE_NDOF);

    double val = bno.getVector(Adafruit_BNO055::VECTOR_EULER)[0];
    

    // If using the gyro, have to delay to allow gyro to start up, otherwise it gets garbage readings
    delay(1000);

    #else
    // Have to start wire since we are not using gyro
    Wire.begin();
    #endif

    
    
    // Turn off sensor with shutoff wired
    // Needed to change address of other sensor
    TOF_left.begin();
    TOF_left.VL53L4CD_Off();

    // Delays to wait for operation to complete
    delay(1000);

    // Begin other sensor and change address
    TOF_front.begin();
    TOF_front.InitSensor();
    TOF_front.VL53L4CD_SetI2CAddress(0x54);

    delay(1000);
    
    // Set the range timings for those that worked best through testing
    TOF_front.VL53L4CD_SetRangeTiming(125, 50);
    TOF_front.VL53L4CD_StartRanging();

    delay(1000);

    TOF_left.VL53L4CD_On();
    TOF_left.InitSensor();
    TOF_left.VL53L4CD_SetRangeTiming(125, 50);
    TOF_left.VL53L4CD_StartRanging();

    delay(2000);

}

/**
 * @brief Test TOF sensor and output to screen
 * 
 */
void test_TOF() {
    int dist = 0;
    bool status = false;
    bool printed = false;

    TEST_FRONT_TOF = true;
    TEST_LEFT_TOF = true;

    if (TEST_FRONT_TOF) {
        status = read_TOF_front(&dist);

        if (status) {
            Serial.print("FRONT: ");
            Serial.print(dist);
            Serial.print(" ");
            printed = true;
        }

        
    }

    if (TEST_LEFT_TOF) {
        status = read_TOF_left(&dist);

        if (status) {
            Serial.print("LEFT: ");
            Serial.print(dist);
            printed = true;
        }
    }

    if (printed) {
        Serial.println("");
    }
    
}

/**
 * @brief Read the passed TOF sensor
 * 
 * @param sensor      the sensor to read as a pointer
 * @param output      value to write to
 * @param status_out  actual error value if passed
 * @return true       sensor values are reported good
 * @return false      sensor values are reported bad
 */
bool read_tof(VL53L4CD* sensor, int* output, int* status_out = nullptr) {
    uint8_t NewDataReady = 0;
    VL53L4CD_Result_t results;
    uint8_t status;

    // Check if data is available
    status = sensor->VL53L4CD_CheckForDataReady(&NewDataReady);
    
    // Write status if it was passed
    if (status_out != nullptr) {
        *status_out = status;
    }

    if (!NewDataReady) {
        return false;
    }

    if ((!status) && (NewDataReady != 0)) {
        // (Mandatory) Clear HW interrupt to restart measurements
        sensor->VL53L4CD_ClearInterrupt();

        // Read measured distance. RangeStatus = 0 means valid data
        sensor->VL53L4CD_GetResult(&results);

        // Set output
        *output = results.distance_mm;

        if (results.range_status != 0) {
            // Write status if it was passed
            if (status_out != nullptr) {
                *status_out = results.range_status;
            }
            return false;
        }

        return true;
    }
    return false;
}

/**
 * @brief Read front sensor
 * 
 * @param output      value to write to
 * @param status_out  actual error value if passed
 * @return true       sensor values are reported good
 * @return false      sensor values are reported bad
 */
bool read_TOF_front(int* output, int* status) {
    return read_tof(&TOF_front, output, status);
}

/**
 * @brief Read left sensor
 * 
 * @param output      value to write to
 * @param status_out  actual error value if passed
 * @return true       sensor values are reported good
 * @return false      sensor values are reported bad
 */
bool read_TOF_left(int* output, int* status) {
    return read_tof(&TOF_left, output, status);
}

#ifdef USE_IR
/**
 * @brief           Construct a new dist sensor::dist sensor object
 * 
 * @param sensor    Which sensor to init, pulls info from distance_sensor array
 */
dist_sensor::dist_sensor(uint8_t sensor) {
    this->val_pin = distance_sensors[sensor][DIST_VAL];
    this->gpio_pin = distance_sensors[sensor][DIST_GPIO];

    pinMode(this->val_pin, INPUT);
    pinMode(this->gpio_pin, OUTPUT);
    digitalWrite(this->gpio_pin, HIGH);

    for (int i = 0; i < 2; ++i) {
        this->cal_distances_mm[i] = distance_mm_vals[sensor][i];
        this->cal_values[i] = 0;
    }
    this->sens_num = sensor;
}

/**
 * @brief       Receive calibration data for use in distance values
 * 
 * @param val1  x1 when linearly interpolating
 * @param val2  y1 when linearly interpolating
 */
void dist_sensor::calibrate(uint16_t val1, uint16_t val2){
    this->cal_values[0] = val1;
    this->cal_values[1] = val2;
}
#endif

#ifdef USE_GYRO
/**
 * @brief           Grab current rotation from sensor, must first request an update
 * 
 * @return float    Rotation in deg, bound to range of -180 to 180
 */
float get_rotation() {
    return bno.getVector(Adafruit_BNO055::VECTOR_EULER).z();
}

/**
 * @brief Return difference between two deg values within -180 to 180
 * 
 * @param current current reading
 * @param target  target value
 * @return float  bound difference
 */
float deg_difference(float current, float target) {
    float diff = target - current;

    // If more than 360, remove until within range
    while(diff > 360) {
        diff = diff - 360;
    }

    while (diff < -360) {
        diff = diff + 360;
    }

    // normalize between 180 and -180
    if (diff > 180) {
        diff = diff - 360;
    } else if (diff < -180) {
        diff = diff + 360;
    }

    Serial.print("Current: ");
    Serial.print(current);
    Serial.print(" Target: ");
    Serial.print(target);
    Serial.print(" Diff: ");
    Serial.println(diff);

    return diff;
}

/**
 * @brief           Add a certain amount of degrees to the passed value,
 *                  binding to a range of -180 to 180
 * 
 * @param degrees   Initial value of degrees
 * @param addition  How many degrees to add
 * @return float    The bound value of the addition
 */
float add_degrees(float degrees, float addition) {
    float new_degrees = degrees + addition;
    
    while(new_degrees > 360) {
        new_degrees = new_degrees - 360;
    }

    while (new_degrees < 0) {
        new_degrees = new_degrees + 360;
    }

    return new_degrees;
}

#endif

