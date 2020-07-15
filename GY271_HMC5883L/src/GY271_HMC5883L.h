/*****************************************************************************/
//  Function:     Header file for HMC5883L
//  Hardware:    Grove - 3-Axis Digital Compass
//  Arduino IDE: Arduino-1.0
//  Author:     Frankie.Chu
//  Date:      Jan 10,2013
//  Version: v1.0
//  by www.seeedstudio.com
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

#ifndef __GY271_HMC5883L_H__
#define __GY271_HMC5883L_H__

#include <Arduino.h>
#include <Wire.h>

#define HMC5883L_ADDRESS            0x1E
#define CONFIGURATION_REGISTERA     0x00
#define CONFIGURATION_REGISTERB     0x01
#define MODE_REGISTER               0x02
#define DATA_REGISTER_BEGIN         0x03

#define MEASUREMENT_CONTINUOUS      0x00
#define MEASUREMENT_SINGLE          0x01
#define MEASUREMENT_IDLE            0x02

#define ERRORCODE_SCALE             1
#define BINARY_NOT_FOUND            -1

struct MagnetometerScaled{
    float XAxis;
    float YAxis;
    float ZAxis;
};

struct MagnetometerRaw{
    short XAxis;
    short YAxis;
    short ZAxis;
};

class HMC5883L{        
    public:
        HMC5883L();

        //void begin(float gain, uint8_t mode);
        void begin(float gain, uint8_t mode, float declination);

        void setGain(float gain);
        void setMeasurementMode(uint8_t mode);
        //void setDeclinationAngle(int degree, int minute, char direction);
        void setDeclinationAngle(float declination);
        
        void initCalibration();
        
        float getGainRange(){return GainRange;}
        float getGainResolution(){return GainResolution;}
        String getModeRegister(){return ModeRegister;}
        float getDeclinationRadians(){return DeclinationRadians;}
        MagnetometerScaled getCalibrationOffsets(){return CalibrationOffsets;}

        MagnetometerRaw getRawAxis();
        MagnetometerScaled getScaledAxis();
        float getCompassDegrees();

        int searchValueIndex(float *array, int start, int end, float value);
        char* getErrorText(int errorCode);
        
    protected:
        void write(short address, short byte);
        uint8_t* read(short address, short length);

    private:
        float gainRanges[8] = {0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1};
        uint8_t gainBits[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        float gainResolutions[8] = {0.73, 0.92, 1.22, 1.52, 2.27, 2.56, 3.03, 4.35};

        float GainRange;
        float GainResolution;
        String ModeRegister;
        float DeclinationRadians;
        MagnetometerScaled CalibrationOffsets;
};

#endif
