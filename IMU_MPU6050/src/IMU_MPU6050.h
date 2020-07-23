#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include "mpu6050_defines.h"

struct IMUBias{
    int32_t GyroX;
    int32_t GyroY;
    int32_t GyroZ;
    int32_t AccelX;
    int32_t AccelY;
    int32_t AccelZ;
};

struct IMURawData{
    int16_t XAxis;
    int16_t YAxis;
    int16_t ZAxis;
};

struct IMUScaledData{
    float XAxis;
    float YAxis;
    float ZAxis;
};

// Set initial input parameters
enum AccelScale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum GyroScale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

class IMU{
    public:
        // CONSTRUCTORS
        IMU();
        IMU(uint8_t address);

        // INITIAL SETTINGS
        void setSensitivity(uint8_t accelScale, uint8_t gyroScale);
        void begin(uint8_t accelScale, uint8_t gyroScale);
        void setSensorSettings(uint8_t address, int scale);
        void lowPowerAccel();

        // SELF-TEST
		//TO-DO: fix the methods that return a pointer
        void test();
        float* selfTestRatio();
        uint8_t* getSelfTestResponse();
        float* getFactoryTrim(uint8_t* selfTest);

        // CALIBRATION
        void calibrate();
        void setCalibrationSettings();
        IMUBias getBias();
        void setGyroOffset(IMUBias bias);
        void setAccelOffset(IMUBias bias);
        
        // SENSOR DATA
        bool interruptStatus(){return (readData(INT_STATUS) & 0x01);}
        IMURawData readRawData(uint8_t address);
        IMURawData getGyroRawData(){return readRawData(GYRO_XOUT_H);}
        IMURawData getAccelRawData(){return readRawData(ACCEL_XOUT_H);}

        IMUScaledData readScaledData(uint8_t address, float sensitivity, IMUScaledData offset);
        IMUScaledData getScaledGyro(){return readScaledData(GYRO_XOUT_H, GyroSensitivity, GyroOffset);}
        IMUScaledData getScaledAccel(){return readScaledData(ACCEL_XOUT_H, AccelSensitivity, AccelOffset);}

        int16_t getTempRawData();
        float getTempCelsius(){return (float(getTempRawData()) / 340.0) + 36.53;}

    protected:
        void writeData(uint8_t address, uint8_t data);
        void readData(uint8_t address, uint8_t *data, uint8_t length);
        uint8_t readData(uint8_t address);

    private:
        uint8_t MPU6050Address;
        int GyroScale;
        int AccelScale;
        float AccelSensitivity;
        float GyroSensitivity;
        
        IMUBias MPU6050Bias;
        IMUScaledData AccelOffset, GyroOffset;

        bool calibrationStatus;
};

#endif