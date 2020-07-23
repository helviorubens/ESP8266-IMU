#include "IMU_MPU6050.h"

// ----- CONSTRUCTORS
IMU::IMU(){
    MPU6050Address = 0x68;
    calibrationStatus = false;
    AccelOffset = {0, 0, 0};
    GyroOffset = {0, 0, 0};
    MPU6050Bias = {0, 0, 0, 0, 0, 0};
}

IMU::IMU(uint8_t address){
    MPU6050Address = address;
    AccelOffset = {0, 0, 0};
    GyroOffset = {0, 0, 0};
    MPU6050Bias = {0, 0, 0, 0, 0, 0};
}

// ----- INITIAL SETTINGS
/** Set Accelerometer and Gyroscope LSB sensitivity
 *  @param accelScale [AFS_2G or AFS_4G or AFS_8G or AFS_16G]
 *  @param gyroScale [GFS_250DPS or GFS_500DPS or GFS_1000DPS or GFS_2000DPS]
 **/
void IMU::setSensitivity(uint8_t accelScale, uint8_t gyroScale){
    // Full range accelerometer LSB Sensitivity in LSB/g
    float accelLSBSensitivity[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    // Full range gyroscope LSB Sensitivity in LSB/degrees/second (LSB/°/s)
    float gyroLSBSensitivity[4] = {131.0, 65.5, 32.8, 16.4};

    // Set current sensor sensitivity
    AccelSensitivity = accelLSBSensitivity[accelScale];
    GyroSensitivity = gyroLSBSensitivity[gyroScale];
}

/** Initialize MPU6050 settings
 **/
void IMU::begin(uint8_t accelScale, uint8_t gyroScale){
    setSensitivity(accelScale, gyroScale);
    
    // If calibration has not been used, wake up device first
    if(!calibrationStatus){
        writeData(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
        delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
    }

    // get stable time source
    writeData(PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 011; this sets the sample rate at 1 kHz for both
    // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
    writeData(CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeData(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set sensor full scale range
    setSensorSettings(GYRO_CONFIG, gyroScale);
    setSensorSettings(ACCEL_CONFIG, accelScale);

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeData(INT_PIN_CFG, 0x22);
    writeData(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

/** Set IMU sensor full scale range
 *  @param address register addres about sensor (ACCEL_CONFIG | GYRO_CONFIG)
 *  @param scale sensor scale (AccelScale | GyroScale)
 **/
void IMU::setSensorSettings(uint8_t address, int scale){
    // 
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readData(address);
    writeData(address, c & ~0xE0); // Clear self-test bits [7:5]
    writeData(address, c & ~0x18); // Clear AFS bits [4:3]
    writeData(address, c | scale << 3); // Set full scale range for the sensor
}

/** Configure the motion detection control for low power accelerometer mode
 **/
void IMU::lowPowerAccel(){
    // The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
    // Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
    // above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
    // threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
    // consideration for these threshold evaluations; otherwise, the flags would be set all the time!

    uint8_t c = readData(PWR_MGMT_1);

    writeData(PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
    writeData(PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

    c = readData(PWR_MGMT_2);
    writeData(PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
    writeData(PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

    c = readData(ACCEL_CONFIG);
    writeData(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
    writeData(ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

    c = readData(CONFIG);
    writeData(CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
    writeData(CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

    c = readData(INT_ENABLE);
    writeData(INT_ENABLE, c & ~0xFF);  // Clear all interrupts
    writeData(INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

    // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
    // for at least the counter duration
    writeData(MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
    writeData(MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

    delay (100);  // Add delay for accumulation of samples

    c = readData(ACCEL_CONFIG);
    writeData(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    writeData(ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

    c = readData(PWR_MGMT_2);
    writeData(PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
    writeData(PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

    c = readData(PWR_MGMT_1);
    writeData(PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
    writeData(PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts
}

void IMU::test(){
    uint8_t mpuAddress = readData(WHO_AM_I_MPU6050);

    if((mpuAddress == 0x68) || (mpuAddress == 0x69)){
        float* selfTestDeviation = selfTestRatio();
        bool conditionSelfTest[6];
        uint8_t count = 0;

        for (uint8_t i = 0; i < 6; i++){
            conditionSelfTest[i] = selfTestDeviation[i] < 1.0f;

            if(!conditionSelfTest[i]){
                Serial.printf("-----STR Deviation [%u] = %.2f DIDN'T PASS the test!\n", i, selfTestDeviation[i]);
            } else{
                Serial.printf("-----STR Deviation [%u] = %.2f PASSED!\n", i, selfTestDeviation[i]);
                count++;
            }
        }

        if(count == 6){
            Serial.println("ALL SELF-TESTS PASSED!");
        } else{
            Serial.println("NOT ALL SELF-TESTS PASSED!");
        }
    } else{
        Serial.print("Could not connect to MPU6050: 0x");
        Serial.println(mpuAddress, HEX);
        while(1); // Loop forever if communication doesn't happen
    }
}

/** Accelerometer and gyroscope self test
 *  check calibration wrt factory settings
 **/
float* IMU::selfTestRatio(){
    // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass  
    float selfTestDeviation[6];

    // Configure the accelerometer for self-test
    writeData(ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeData(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(250);  // Delay a while to let the device execute the self-test

    uint8_t* selfTest = getSelfTestResponse(); // Calcule Self Test Response (STR)
    float* factoryTrim = getFactoryTrim(selfTest); // Calcule Factory Trim (FT) values

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        selfTestDeviation[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
    }

    return selfTestDeviation;
}

uint8_t* IMU::getSelfTestResponse(){
    uint8_t rawData[4], selfTest[6];

    rawData[0] = readData(SELF_TEST_X); // X-axis self-test results
    rawData[1] = readData(SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = readData(SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = readData(SELF_TEST_A); // Mixed-axis self-test results

    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer

    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    
    return selfTest;
}

float* IMU::getFactoryTrim(uint8_t* selfTest){
    float factoryTrim[6];

    factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation

    factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

    return factoryTrim;
}

// ----- CALIBRATION METHODS
/** Calibrate MPU6050
 *  Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
 * of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
 **/
void IMU::calibrate(){
    IMUBias sensorBias = IMUBias();

    setCalibrationSettings();

    sensorBias = getBias();

    setGyroOffset(sensorBias);

    setAccelOffset(sensorBias);

    calibrationStatus = true;
}

void IMU::setCalibrationSettings(){
    // Set accelerometer sensitivity to 16384 LSB/g (scale 2g) and gyroscope sensitivity to 131 LSB/°/s (scale 250°/s)
    setSensitivity(AFS_2G, GFS_250DPS);

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeData(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeData(PWR_MGMT_1, 0x01);
    writeData(PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    writeData(INT_ENABLE, 0x00);   // Disable all interrupts
    writeData(FIFO_EN, 0x00);      // Disable FIFO
    writeData(PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeData(I2C_MST_CTRL, 0x00); // Disable I2C master
    writeData(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeData(USER_CTRL, 0x0C);    // Reset FIFO and DMP (00001100)
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeData(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeData(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeData(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeData(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
}

IMUBias IMU::getBias(){
    IMUBias bias = IMUBias(); // zero set initialization
    uint8_t data[12] = {0}; // data array to hold accelerometer and gyro x, y, z, data
    uint8_t dataSensor[12];

    int32_t biasAuxiliary[6] = {0, 0, 0, 0, 0, 0};

    uint16_t fifo_count, packet_count;

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeData(USER_CTRL, 0x40);   // Enable FIFO
    writeData(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeData(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readData(FIFO_COUNTH, data, 2); // read FIFO sample count

    fifo_count = ((uint16_t)data[0] << 8) | data[1];

    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging
    
    for (uint16_t ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readData(FIFO_R_W, data, 12); // read data for averaging

        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;

        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
        
        // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        biasAuxiliary[0] += (int32_t) accel_temp[0]; 
        biasAuxiliary[1] += (int32_t) accel_temp[1];
        biasAuxiliary[2] += (int32_t) accel_temp[2];
        biasAuxiliary[3] += (int32_t) gyro_temp[0];
        biasAuxiliary[4] += (int32_t) gyro_temp[1];
        biasAuxiliary[5] += (int32_t) gyro_temp[2];
    }

    // Normalize sums to get average count biases
    for (uint8_t i = 0; i < 6; i++){
        biasAuxiliary[i] /= (int32_t) packet_count;
    }

    if (biasAuxiliary[2] > 0L) {
        biasAuxiliary[2] -= (int32_t) AccelSensitivity; // Remove gravity from the z-axis accelerometer bias calculation
    }
    else {
        biasAuxiliary[2] += (int32_t) AccelSensitivity;
    }

    // Set bia struct object with bias auxiliary array
    bias.AccelX = biasAuxiliary[0];
    bias.AccelY = biasAuxiliary[1];
    bias.AccelZ = biasAuxiliary[2];
    bias.GyroX  = biasAuxiliary[3];
    bias.GyroY  = biasAuxiliary[4];
    bias.GyroZ  = biasAuxiliary[5];

    return bias;
}

void IMU::setGyroOffset(IMUBias bias){
    uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
    int32_t gyroBias[3] = {bias.GyroX, bias.GyroY, bias.GyroZ};

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyroBias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyroBias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyroBias[1] / 4  >> 8) & 0xFF;
    data[3] = (-gyroBias[1] / 4)       & 0xFF;
    data[4] = (-gyroBias[2] / 4  >> 8) & 0xFF;
    data[5] = (-gyroBias[2] / 4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeData(XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
    writeData(XG_OFFS_USRL, data[1]);
    writeData(YG_OFFS_USRH, data[2]);
    writeData(YG_OFFS_USRL, data[3]);
    writeData(ZG_OFFS_USRH, data[4]);
    writeData(ZG_OFFS_USRL, data[5]);

    // construct gyro bias in deg/s for later manual subtraction
    GyroOffset.XAxis = (float) gyroBias[0] /  GyroSensitivity;
    GyroOffset.YAxis = (float) gyroBias[1] /  GyroSensitivity;
    GyroOffset.ZAxis = (float) gyroBias[2] /  GyroSensitivity;
}

void IMU::setAccelOffset(IMUBias bias){
    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
    uint8_t data[6] = {0}; // data array to hold accelerometer and gyro x, y, z, data
    int32_t accelBias[3] = {bias.AccelX, bias.AccelY, bias.AccelZ};
    int32_t accelBias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases

    readData(XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accelBias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    readData(YA_OFFSET_H, data, 2);
    accelBias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    readData(ZA_OFFSET_H, data, 2);
    accelBias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (uint16_t ii = 0; ii < 3; ii++) {
        if (accelBias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accelBias_reg[0] -= (accelBias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accelBias_reg[1] -= (accelBias[1] / 8);
    accelBias_reg[2] -= (accelBias[2] / 8);

    data[0] = (accelBias_reg[0] >> 8) & 0xFF;
    data[1] = (accelBias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accelBias_reg[1] >> 8) & 0xFF;
    data[3] = (accelBias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accelBias_reg[2] >> 8) & 0xFF;
    data[5] = (accelBias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    writeData(XA_OFFSET_H, data[0]); // might not be supported in MPU6050
    writeData(XA_OFFSET_L_TC, data[1]);
    writeData(YA_OFFSET_H, data[2]);
    writeData(YA_OFFSET_L_TC, data[3]);
    writeData(ZA_OFFSET_H, data[4]);
    writeData(ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    AccelOffset.XAxis = (float)accelBias[0] / AccelSensitivity;
    AccelOffset.YAxis = (float)accelBias[1] / AccelSensitivity;
    AccelOffset.ZAxis = (float)accelBias[2] / AccelSensitivity;
}

// ----- SENSOR DATA
IMURawData IMU::readRawData(uint8_t address){
    IMURawData raw = IMURawData(); // zero initialized x/y/z register. Data stored here
    uint8_t rawData[6] = {0};

    readData(address, rawData, 6); // Read the six raw data registers into data array

    raw.XAxis = (int16_t)((rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
    raw.YAxis = (int16_t)((rawData[2] << 8) | rawData[3]);
    raw.ZAxis = (int16_t)((rawData[4] << 8) | rawData[5]);

    return raw;
}

IMUScaledData IMU::readScaledData(uint8_t address, float sensitivity, IMUScaledData offset){
    IMURawData raw = readRawData(address);
    IMUScaledData scaled = IMUScaledData();

    scaled.XAxis = (float)raw.XAxis / sensitivity - offset.XAxis;
    scaled.YAxis = (float)raw.YAxis / sensitivity - offset.YAxis;
    scaled.ZAxis = (float)raw.ZAxis / sensitivity - offset.ZAxis;

    return scaled;
}

int16_t IMU::getTempRawData(){
    // Read the two raw data registers sequentially into data array
    uint8_t rawData[6] = {0};
    readData(TEMP_OUT_H, rawData, 2);  // temperature register data stored here

    return ((int16_t)rawData[0]) << 8 | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

// ----- READ/WRITE METHODS

void IMU::writeData(uint8_t address, uint8_t data){
    Wire.beginTransmission(MPU6050Address);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission(true);
}

void IMU::readData(uint8_t address, uint8_t *data, uint8_t length){
    uint8_t i;

    data[length] = {0};

    Wire.beginTransmission(MPU6050Address); // Initialize the Tx buffer
    Wire.write(address); // Put slave register address in Tx buffer
    Wire.endTransmission(false); // Send false the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(MPU6050Address, length); // Read bytes from slave register address

    if(Wire.available() == length){
        for(i = 0; i < length; i++){
            data[i] = Wire.read(); // Put read results in the Rx buffer
        }
    }

    Wire.endTransmission(true);
}

uint8_t IMU::readData(uint8_t address){
    uint8_t data[1] = {0};

    readData(address, data, 1);

    return data[0];
}
