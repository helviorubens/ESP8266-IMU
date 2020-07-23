#include <IMU_MPU6050.h>

const uint8_t INTERRUPT_PIN = 13;               // GPIO13/D7 on ESP8266
const uint8_t SDA_PIN = 4;                      // GPIO4/D2  on ESP8266
const uint8_t SCL_PIN = 5;                      // GPIO5/D1  on ESP8266

uint8_t ACCEL_SCALE = AFS_2G; // [AFS_2G or AFS_4G or AFS_8G or AFS_16G]
uint8_t GYRO_SCALE = GFS_250DPS; // [GFS_250DPS or GFS_500DPS or GFS_1000DPS or GFS_2000DPS]

uint32_t INTERVAL = 100L, DELTA_TIME = 0L, PREVIOUS_TIME = 0L;

IMUScaledData SCALED_GYRO, SCALED_ACCEL;
IMURawData GYRO_RAW, ACCEL_RAW;
float TEMPERATURE;

IMU mpu6050;

void setup(){
    Serial.begin(115200);
    Wire.begin();
    delay(100);

    Serial.println("\n[START SETUP]");
    pinMode(INTERRUPT_PIN, INPUT);
    
    Serial.println("\t-----[BEGIN CALIBRATION]");
    // Calibrate gyro and accelerometers, load biases in bias registers and set offset values
    mpu6050.calibrate();
    Serial.println("\t-----[END CALIBRATION");

    Serial.println("\t-----[BEGIN MPU6050]");
    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    mpu6050.begin(ACCEL_SCALE, GYRO_SCALE);
    Serial.println("-----[END SETUP]");
}

void loop(){
    // If data ready bit set, all data registers have new data
    if(mpu6050.interruptStatus()){ // check if data ready interrupt
        //GYRO_RAW = mpu6050.getGyroRawData();
        //ACCEL_RAW = mpu6050.getAccelRawData();

        // Calculate the gyro value into actual degrees per second
        SCALED_GYRO = mpu6050.getScaledGyro();

        // Now we'll calculate the accleration value into actual g's
        SCALED_ACCEL = mpu6050.getScaledAccel();

        // Temperature in Celsius degrees
        TEMPERATURE = mpu6050.getTempCelsius();
    }

    DELTA_TIME = millis() - PREVIOUS_TIME;

    if(DELTA_TIME > INTERVAL){
        Serial.print("\nTemp\t");
        // Print temperature in Celsius degrees
        Serial.print(TEMPERATURE, 2);Serial.println(" °C");
      
        Serial.print("Accel\t(");
        // Print acceleration values in milligs!
        Serial.print(1000*SCALED_ACCEL.XAxis);Serial.print(" | ");
        Serial.print(1000*SCALED_ACCEL.YAxis);Serial.print(" | ");
        Serial.print(1000*SCALED_ACCEL.ZAxis);Serial.println(") mg");
        
        Serial.print("Gyro\t(");
        // Print gyro values in degree/sec
        Serial.print(SCALED_GYRO.XAxis, 2);Serial.print(" | ");
        Serial.print(SCALED_GYRO.YAxis, 2);Serial.print(" | ");
        Serial.print(SCALED_GYRO.ZAxis, 2);Serial.println(") deg/s");

        

        PREVIOUS_TIME = millis();
    }
}
