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

enum FieldRange {
    GN_0_88GA = 0,
    GN_1_3GA,
    GN_1_9GA,
    GN_2_5GA,
    GN_4_0GA,
    GN_4_7GA,
    GN_5_6GA,
    GN_8_1GA
};

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
        // CONSTRUCTORS
        HMC5883L();

        // INITIAL SETTINGS
        void begin(uint8_t range, uint8_t mode, float declination);

        void setGain(uint8_t range);
        void setMeasurementMode(uint8_t mode);
        void setDeclinationAngle(float declination);
        
        // CALIBRATION
        void calibrate();
        
        float getGainResolution(){return GainResolution;}
        String getModeRegister(){return ModeRegister;}
        float getDeclinationRadians(){return DeclinationRadians;}
        MagnetometerScaled getCalibrationOffsets(){return CalibrationOffsets;}

        // MEASUREMENTS
        MagnetometerRaw getRawAxis();
        MagnetometerScaled getScaledAxis();
        float getCompassDegrees();
        
    protected:
        // COMMUNICATION
        void write(int address, uint8_t data);
        void read(int address, uint8_t *data, int length);
        uint8_t read(int address);

    private:
        float GainResolution;
        String ModeRegister;
        float DeclinationRadians;
        MagnetometerScaled CalibrationOffsets;
};

#endif
