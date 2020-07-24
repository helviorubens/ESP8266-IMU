/** GY-271 (HMC5883L) 3-Axis Magnetometer/Digital Compass
 * Author: Hélvio Albuquerque (rewritten from Grove_3Axis_Digital_Compass_HMC5883L library. See README for more informations)
 * Function: Get the geographic direction of the X-axis as digital compass (0° points to the NORTH)
 * 
 * Hardware setup:
 * GY-271/HMC5883L ---------- Arduino --------- NODEMCU/ESP8266
 * VCC ---------------------- 5V -------------- Vin (5V)
 * SDA ---------------------- A4 -------------- D2(GPIO4)
 * SCL ---------------------- A5 -------------- D1(GPIO5)
 * GND ---------------------- GND ------------- GND
 **/
#include "GY271_HMC5883L.h"

const uint8_t LED_PIN = 2;                          // Use PIN 2 (D4 or BUILTIN_LED) on ESP8266

// Sensor field ranges: GN_0_88GA, GN_1_3GA, GN_1_9GA, GN_2_5GA, GN_4_0GA, GN_4_7GA, GN_5_6GA, GN_8_1GA
uint8_t SENSOR_FIELD = GN_1_9GA;
uint8_t MEASUREMENT = MEASUREMENT_CONTINUOUS;       // MEASUREMENT_CONTINUOUS / MEASUREMENT_SINGLE / MEASUREMENT_IDLE
/** 
 *  Find your declination angle here: http://www.magnetic-declination.com/
 *  Campina Grande, Paraíba, Brazil | Latitude: 7° 13' 50" S | Longitude: 35° 52' 52" W 
 *  Mine is: -21°36' [WEST] -- USE THIS --> DECLINATION = -21.36
 **/
float DECLINATION = -21.36;

// Store our compass 
HMC5883L compass;

// Functions prototype
void printGain(uint8_t field);

// Out setup routine, here we will configure the microcontroller and compass.
void setup(){
    // Initialize the serial port.
    Serial.begin(115200);
    delay(100);

    Serial.println("-----[STARTING I2C COMMUNICATION]");
    Wire.begin();
    delay(100);

    Serial.println("-----[STARTING HMC5883L]");
    compass.begin(SENSOR_FIELD, MEASUREMENT, DECLINATION);

    delay(500);

    Serial.print("\t-----[Gain Range]: ");
    printGain(SENSOR_FIELD);
    //Serial.print(compass.getGainRange()); Serial.println(" Ga");
    Serial.print("\t-----[Gain Resolution]: ");
    Serial.print(compass.getGainResolution()); Serial.println(" mGa/LSb");

    Serial.print("\t-----[Measurement Mode]: ");
    Serial.println(compass.getModeRegister());

    Serial.print("\t-----[Declination]: ");
    Serial.print(compass.getDeclinationRadians()); Serial.println(" radians");

    delay(1000);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("-----[STARTING CALIBRATION]");
    Serial.println("\t-----[Please rotate the compass 3x around X, Y and Z axis]");

    compass.calibrate();

    MagnetometerScaled compassOffset = compass.getCalibrationOffsets();

    Serial.print("\t-----[Offsets X-Y-Z]: ");
    Serial.print(compassOffset.XAxis);Serial.print(" | ");
    Serial.print(compassOffset.YAxis);Serial.print(" | ");
    Serial.println(compassOffset.ZAxis);

    Serial.println("-----[ENDING CALIBRATION]");

    digitalWrite(LED_PIN, HIGH);

    Serial.println("\n-----[START MEASUREMENTS]");
}

void loop(){
    Serial.print("\n-----[ANGLE (degrees)]: ");
    Serial.println(compass.getCompassDegrees());

    // Normally we would delay the application by 66ms to allow the loop
    // to run at 15Hz (default bandwidth for the HMC5883L).
    // However since we have a long serial out (104ms at 9600) we will let
    // it run at its natural speed.
    delay(66);//of course it can be delayed longer.
}

void printGain(uint8_t field){
    switch (field){
        case GN_0_88GA:
            Serial.print("0.88 Ga");
            break;
        case GN_1_3GA:
            Serial.print("1.3 Ga");
            break;
        case GN_1_9GA:
            Serial.print("1.9 Ga");
            break;
        case GN_2_5GA:
            Serial.print("2.5 Ga");
            break;
        case GN_4_0GA:
            Serial.print("4.0 Ga");
            break;
        case GN_4_7GA:
            Serial.print("4.7 Ga");
            break;
        case GN_5_6GA:
            Serial.print("5.6 Ga");
            break;
        case GN_8_1GA:
            Serial.print("8.1 Ga");
            break;
        default:
            Serial.print("Not defined!");
            break;
    }
}