/*****************************************************************************/	
//	Function:	 Get the Geographic direction of the X-axis.
//				If X-axis points to the North, it is 0 degree.
//				If X-axis points to the East, it is 90 degrees.
//				If X-axis points to the South, it is 180 degrees.
//				If X-axis points to the West, it is 270 degrees.
//  Hardware:   Grove - 3-Axis Digital Compass
//	Arduino IDE: Arduino-1.0
//	Author:	 Frankie.Chu		
//	Date: 	 Jan 10,2013
//	Version: v1.0
//
//  Modified by: Yihui Xiong
//  Data:        June 19, 2013
//  Description: add calibrate function
//
//  Modified by: Helvio Albuquerque
//  Data:        July 11, 2020
//  Description: add/change class functions
//
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include <Wire.h>
#include "GY271_HMC5883L.h"

//const uint8_t SDA_PIN = 4;                          // use pin 4 (D2) on ESP8266
//const uint8_t SCL_PIN = 5;                          // use pin 5 (D1) on ESP8266
const uint8_t LED_PIN = 2;                          // Use PIN 2 (D4 or BUILTIN_LED) on ESP8266

float SENSOR_GAIN = 1.9;                            // GAINS: 0.88 / 1.3 / 1.9 / 2.5 / 4.0 / 4.7 / 5.6 / 8.1
uint8_t MEASUREMENT = MEASUREMENT_CONTINUOUS;       // MEASUREMENT_CONTINUOUS / MEASUREMENT_SINGLE / MEASUREMENT_IDLE
/** 
 *  Find your declination angle here: http://www.magnetic-declination.com/
 *  Campina Grande, Paraíba, Brazil | Latitude: 7° 13' 50" S | Longitude: 35° 52' 52" W 
 *  Mine is: -21°36' [WEST] -- USE THIS --> DECLINATION = -21.36
 **/
float DECLINATION = -21.36;
//int DECLINATION_DEGREES = 21;
//int DECLINATION_MINUTES = 36;
//char DECLINATION_DIRECTION = 'W';

// Store our compass 
HMC5883L compass;

// Out setup routine, here we will configure the microcontroller and compass.
void setup(){
    // Initialize the serial port.
    Serial.begin(115200);
    delay(100);

    Serial.println("-----[STARTING I2C COMMUNICATION]");
    Wire.begin();
    delay(100);

    Serial.println("-----[STARTING HMC5883L]");
    //compass.begin(SENSOR_GAIN, MEASUREMENT);
    compass.begin(SENSOR_GAIN, MEASUREMENT, DECLINATION);
    //compass.setDeclinationAngle(DECLINATION_DEGREES, DECLINATION_MINUTES, DECLINATION_DIRECTION);

    delay(500);

    Serial.print("\t-----[Gain Range]: ");
    Serial.print(compass.getGainRange()); Serial.println(" Ga");
    Serial.print("\t-----[Gain Resolution]: ");
    Serial.print(compass.getGainResolution()); Serial.println(" mGa/LSb");

    Serial.print("\t-----[Measurement Mode]: ");
    Serial.println(compass.getModeRegister());

    Serial.print("\t-----[Declination]: ");
//    Serial.print(int(DECLINATION));Serial.print("°");
//    Serial.print(abs((DECLINATION - int(DECLINATION))*100));Serial.print("\']: ");
//    Serial.print(String(DECLINATION_DEGREES)+"°");
//    Serial.print(String(DECLINATION_MINUTES)+"\' (");
//    Serial.print(String(DECLINATION_DIRECTION)+")]: ");
    Serial.print(compass.getDeclinationRadians()); Serial.println(" radians");

    delay(1000);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("-----[STARTING CALIBRATION]");
    Serial.println("\t-----[Please rotate the compass 3x around X, Y and Z axis]");

    compass.initCalibration();

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
    /*
    MagnetometerRaw raw = compass.getRawAxis();
    Serial.print("\n-----[RAW X-Y-Z]: ");
    Serial.print(raw.XAxis); Serial.print(" | ");
    Serial.print(raw.YAxis); Serial.print(" | ");
    Serial.println(raw.ZAxis);
    */

    //Serial.print("\n-----[SCALED X-Y-Z]: ");
    //Serial.println(compass.compass.getScaledAxis());

    Serial.print("\n-----[ANGLE (degrees)]: ");
    Serial.println(compass.getCompassDegrees());

    // Normally we would delay the application by 66ms to allow the loop
    // to run at 15Hz (default bandwidth for the HMC5883L).
    // However since we have a long serial out (104ms at 9600) we will let
    // it run at its natural speed.
    delay(66);//of course it can be delayed longer.
}
