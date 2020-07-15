/*****************************************************************************/
//    Function:     Cpp file for HMC5883L
//  Hardware:    Grove - 3-Axis Digital Compass
//    Arduino IDE: Arduino-1.0
//    Author:     FrankieChu
//    Date:      Jan 10,2013
//    Version: v1.0
//    by www.seeedstudio.com
//
//  Modified by: Helvio Albuquerque
//  Data:        July 11, 2020
//  Description: change classes
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

#include <Arduino.h>
#include "GY271_HMC5883L.h"

/** Default constructor, uses scale +/- 1.3 Ga e MEASUREMENT_CONTINUOUS.
 * 
 */
HMC5883L::HMC5883L(){
    GainRange = 1.3;
    GainResolution = 0.92;
    ModeRegister = "CONTINUOUS";
    DeclinationRadians = 0.0;
    CalibrationOffsets = {0, 0, 0};
}

/** Specific scale and mode constructor
 * @param scale set magnetic field gain value (0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1)
 * @param mode set measurement mode (MEASUREMENT_CONTINUOUS, MEASUREMENT_SINGLE, MEASUREMENT_IDLE)
 * @see setScale()
 * @see setMeasurementMode()
 */
/*
void HMC5883L::begin(float gain, uint8_t mode){
    setGain(gain);
    setMeasurementMode(mode);
}
*/

void HMC5883L::begin(float gain, uint8_t mode, float declination){
    setGain(gain);
    setMeasurementMode(mode);
    setDeclinationAngle(declination);
}


void HMC5883L::write(short address, short data)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t* HMC5883L::read(short address, short length)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, length);

    uint8_t buffer[length];
    
    if(Wire.available() == length)
    {
        for(uint8_t i = 0; i < length; i++)
        {
            buffer[i] = Wire.read();
        }
    }
    
    Wire.endTransmission();
    return buffer;
}

/** Set magnetic field gain value.
 * @param gain magnetic field gain value (0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1)
 */
void HMC5883L::setGain(float gain){
    uint8_t regValue = 0x00;
    int vectorSize = sizeof(gainRanges)/sizeof(gainRanges[0]);
    
    Serial.print("\t-----[vectorSize]: ");
    Serial.println(vectorSize);

    int binaryIndex = searchValueIndex(gainRanges, 0, vectorSize-1, gain);

    Serial.print("\t-----[binaryIndex]: ");
    Serial.println(binaryIndex);

    if((binaryIndex >= 0) && (binaryIndex <= vectorSize-1)){
        GainRange = gain;
        GainResolution = gainResolutions[binaryIndex];
        regValue = gainBits[binaryIndex];
    } else{
        // Default value field range: +/- 1.3 Ga and digital resolution 0.92 mG/LSb
        GainRange = 1.3;
        GainResolution = 0.92;
        regValue = 0x01;
    }

    // Setting is in the top 3 bits of the register.
    regValue = regValue << 5;
    write(CONFIGURATION_REGISTERB, regValue);
}

/** Set measurement mode register
 *  @param mode (MEASUREMENT_CONTINUOUS or MEASUREMENT_SINGLE or MEASUREMENT_IDLE)
 */
void HMC5883L::setMeasurementMode(uint8_t mode){
    write(MODE_REGISTER, mode);

    switch(mode){
    case MEASUREMENT_CONTINUOUS:
        ModeRegister = "CONTINUOUS";
        break;
    case MEASUREMENT_SINGLE:
        ModeRegister = "SINGLE";
        break;
    case MEASUREMENT_IDLE:
        ModeRegister = "IDLE";
        break;    
    default:
        ModeRegister = "UNDEFINED";
        break;
    }
}

/** Set declination angle (D°M' WEST/EAST) in radians
 * @param degree first number in degree (D°)
 * @param minute second number in minutes (M')
 * @param direction direction 'W' (WEST) or 'E' (EAST)
 * @see http://www.magnetic-declination.com/
 */
/*
void HMC5883L::setDeclinationAngle(int degree, int minute, char direction){
    switch (direction){
    case 'E':
        DeclinationRadians = (float(abs(degree)) + float(minute/60.0)) * (M_PI / 180.0);
        break;
    case 'W':
        DeclinationRadians = 0.0 - (float(abs(degree)) + float(minute/60.0)) * (M_PI / 180.0);
        break;
    default:
        DeclinationRadians = 0.0;
        break;
    }
}
*/

void HMC5883L::setDeclinationAngle(float declination){
    int degree = (int) declination;
    float minutes = (declination - degree) / 60.0;

    DeclinationRadians = float(degree + minutes) * (M_PI / 180.0);
}

/** Begin compass calibration
 *  @return nothing
 */
void HMC5883L::initCalibration(){
    MagnetometerScaled valueMax = {0, 0, 0};
    MagnetometerScaled valueMin = {0, 0, 0};
    int xcount = 0;
    int ycount = 0;
    int zcount = 0;
    boolean xZero = false;
    boolean yZero = false;
    boolean zZero = false;
    MagnetometerScaled value;    

    while (xcount < 3 || ycount < 3 || zcount < 3) {
        value = getScaledAxis();
        
        /*
        Serial.print("\t-----[xcount | ycount | zcount]: ");
        Serial.print(xcount); Serial.print(" | ");
        Serial.print(ycount); Serial.print(" | ");
        Serial.println(zcount);

        Serial.print("\t-----[xZero | yZero | zZero]: ");
        Serial.print(xZero); Serial.print(" | ");
        Serial.print(yZero); Serial.print(" | ");
        Serial.println(zZero);
        
        Serial.print("\t-----[value XAxis | YAxis | ZAxis]: ");
        Serial.print(value.XAxis); Serial.print(" | ");
        Serial.print(value.YAxis); Serial.print(" | ");
        Serial.println(value.ZAxis);
        */

        if ((fabs(value.XAxis) > 600) || (fabs(value.YAxis) > 600) || (fabs(value.ZAxis) > 600)) {
            continue;
        }

        if (valueMin.XAxis > value.XAxis) {
            valueMin.XAxis = value.XAxis;
        } else if (valueMax.XAxis < value.XAxis) {
            valueMax.XAxis = value.XAxis;
        }

        if (valueMin.YAxis > value.YAxis) {
            valueMin.YAxis = value.YAxis;
        } else if (valueMax.YAxis < value.YAxis) {
            valueMax.YAxis = value.YAxis;
        }

        if (valueMin.ZAxis > value.ZAxis) {
            valueMin.ZAxis = value.ZAxis;
        } else if (valueMax.ZAxis < value.ZAxis) {
            valueMax.ZAxis = value.ZAxis;
        }

        if (xZero) {
            if (fabs(value.XAxis) > 50) {
                xZero = false;
                xcount++;
            }
        } else {
            if (fabs(value.XAxis) < 40) {
                xZero = true;
            }
        }

        if (yZero) {
            if (fabs(value.YAxis) > 50) {
                yZero = false;
                ycount++;
            }
        } else {
            if (fabs(value.YAxis) < 40) {
                yZero = true;
            }
        }

        if (zZero) {
            if (fabs(value.ZAxis) > 50) {
                zZero = false;
                zcount++;
            }
        } else {
            if (fabs(value.ZAxis) < 40) {
                zZero = true;
            }
        }

        delay(50);
    }

    CalibrationOffsets.XAxis = (valueMax.XAxis + valueMin.XAxis) / 2.0;
    CalibrationOffsets.YAxis = (valueMax.YAxis + valueMin.YAxis) / 2.0;
    CalibrationOffsets.ZAxis = (valueMax.ZAxis + valueMin.ZAxis) / 2.0;
}

MagnetometerRaw HMC5883L::getRawAxis(){
    uint8_t* buffer = read(DATA_REGISTER_BEGIN, 6);
    MagnetometerRaw raw = MagnetometerRaw();
    raw.XAxis = (buffer[0] << 8) | buffer[1];
    raw.ZAxis = (buffer[2] << 8) | buffer[3];
    raw.YAxis = (buffer[4] << 8) | buffer[5];
    return raw;
}

MagnetometerScaled HMC5883L::getScaledAxis(){
    MagnetometerRaw raw = getRawAxis();
    MagnetometerScaled scaled = MagnetometerScaled();
    scaled.XAxis = raw.XAxis * GainResolution;
    scaled.ZAxis = raw.ZAxis * GainResolution;
    scaled.YAxis = raw.YAxis * GainResolution;

    return scaled;
}

float HMC5883L::getCompassDegrees(){
    // Retrived the scaled values from the compass (scaled to the configured scale).
    MagnetometerScaled scaled = getScaledAxis();

    scaled.XAxis -= CalibrationOffsets.XAxis;
    scaled.YAxis -= CalibrationOffsets.YAxis;
    scaled.ZAxis -= CalibrationOffsets.ZAxis;

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(scaled.YAxis, scaled.XAxis);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location
    heading += DeclinationRadians;

    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
        heading -= 2*PI;

    // Convert to degrees for readability.
    return heading * (180/M_PI);
}

/** Recursive binary search
 * @return index of the searched value or error code
 * @param array ordered vector which will be the search
 * @param start initial search index
 * @param end final search index
 * @param value value to be found
 **/
int HMC5883L::searchValueIndex(float *array, int start, int end, float value){
    int searchIndex = 0;
    bool conditionStart = value > array[start];
    bool conditionEnd = value < array[end];

    if (conditionStart && conditionEnd) {
        searchIndex = (start + end) / 2;
        
        if (array[searchIndex] == value){
            return searchIndex;
        } else if (array[searchIndex] < value){
            return searchValueIndex(array, searchIndex + 1, end, value);
        } else {
            return searchValueIndex(array, start, end - 1, value);
        }
    } else {
        return BINARY_NOT_FOUND;
    }
}

char* HMC5883L::getErrorText(int errorCode){
    switch (errorCode){
    case ERRORCODE_SCALE:
        return "Scale value is not valid! The default value was chosen: 1.3 Ga. If you want to change it, please choose a proper scale value: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1";
        break;
    case BINARY_NOT_FOUND:
        return "Nothing found in binary search!";
        break;
    default:
        return "Error not defined";
        break;
    }
}
