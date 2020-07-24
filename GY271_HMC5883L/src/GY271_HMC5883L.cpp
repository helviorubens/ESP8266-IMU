#include "GY271_HMC5883L.h"

// ----- CONSTRUCTORS
/** Default constructor, uses scale +/- 1.3 Ga e MEASUREMENT_CONTINUOUS by default
 * 
 */
HMC5883L::HMC5883L(){
    GainResolution = 0.92;
    ModeRegister = "CONTINUOUS";
    DeclinationRadians = 0.0;
    CalibrationOffsets = {0, 0, 0};
}

// ----- INITIAL SETTINGS
/** Initialize the main settings about sensor
 * @param gain set magnetic field gain value (0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1)
 * @param mode set measurement mode (MEASUREMENT_CONTINUOUS, MEASUREMENT_SINGLE, MEASUREMENT_IDLE)
 * @param declination set declination angle in format [degree.minutes]
 */
void HMC5883L::begin(uint8_t range, uint8_t mode, float declination){
    setGain(range);
    setMeasurementMode(mode);
    setDeclinationAngle(declination);
}

/** Set magnetic field gain value.
 * @param gain magnetic field gain value (0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1)
 */
void HMC5883L::setGain(uint8_t range){
    uint8_t gainBits[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    float digitalResolutions[8] = {0.73, 0.92, 1.22, 1.52, 2.27, 2.56, 3.03, 4.35};

    GainResolution = digitalResolutions[range];

    uint8_t regValue = range;

    /*
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
    }*/

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
void HMC5883L::setDeclinationAngle(float declination){
    int degree = (int) declination;
    float minutes = (declination - degree) / 60.0;

    DeclinationRadians = float(degree + minutes) * (M_PI / 180.0);
}

// ----- CALIBRATION
/** Begin compass calibration
 *  @return nothing
 */
void HMC5883L::calibrate(){
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

// ----- MEASUREMENTS
MagnetometerRaw HMC5883L::getRawAxis(){
    MagnetometerRaw raw = MagnetometerRaw();
    uint8_t buffer[6] = {0};

    read(DATA_REGISTER_BEGIN, buffer, 6);

    raw.XAxis = (buffer[0] << 8) | buffer[1];
    raw.ZAxis = (buffer[2] << 8) | buffer[3];
    raw.YAxis = (buffer[4] << 8) | buffer[5];

    return raw;
}

MagnetometerScaled HMC5883L::getScaledAxis(){
    MagnetometerRaw raw = getRawAxis();
    MagnetometerScaled scaled = MagnetometerScaled(); // zero set

    scaled.XAxis = raw.XAxis * GainResolution;
    scaled.ZAxis = raw.ZAxis * GainResolution;
    scaled.YAxis = raw.YAxis * GainResolution;

    return scaled;
}

float HMC5883L::getCompassDegrees(){
    // Retrived the scaled values from the compass (scaled to the configured scale).
    MagnetometerScaled scaled = getScaledAxis();
    float heading = 0.0;

    scaled.XAxis -= CalibrationOffsets.XAxis;
    scaled.YAxis -= CalibrationOffsets.YAxis;
    scaled.ZAxis -= CalibrationOffsets.ZAxis;

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    heading = atan2(scaled.YAxis, scaled.XAxis);

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

// ----- COMMUNICATION
void HMC5883L::write(int address, uint8_t data)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

void HMC5883L::read(int address, uint8_t *data, int length)
{
    data[length] = {0};

    Wire.beginTransmission(HMC5883L_ADDRESS);   // Initialize the Tx buffer
    Wire.write(address);    // Put slave register address in Tx buffer
    Wire.endTransmission(false);    // Send false the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(HMC5883L_ADDRESS, length); // Read bytes from slave register address
    
    if(Wire.available() == length){
        for(uint8_t i = 0; i < length; i++){
            data[i] = Wire.read(); // Put read results in the Rx buffer
        }
    }
    
    Wire.endTransmission(true);
}

uint8_t HMC5883L::read(int address){
    uint8_t data[1] = {0};

    read(address, data, 1);

    return data[0];
}