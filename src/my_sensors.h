#ifndef MY_SENSORS_H
#define MY_SENSORS_H

#include <Wire.h>
#include <Arduino.h>


/*=========================== Sensor addresses ===========================*/
#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
#define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
#define L3GD20_ADDRESS                (0x69)              // L3gD20 I2C address; 1101011 in binary
#define BMP085_ADDRESS                (0x77)              
/*========================================================================*/


// Oversampling setting (0..3)
#define OSS 2

// Sea level pressure used as the altitude reference, in Pa
#define SEA_LEVEL_PRESSURE 101325.0


/* Sensors initializing */
void initializeSensors();

/* Zeroes the gyro off 2000 samples and reads the barometer calibration
   table. Copter has to be standing still. Takes about 8 seconds. */
void calibrateSensors();

/* Accelerometer reading */
void readAccel(float &ax, float &ay, float &az);

/* Magnetometer reading */
void readMag(float &mx, float &my, float &mz);

/* Gyroscope reading */
void readGyro(float &gx, float &gy, float &gz);

/* Altitude. Doesn't block - moves a three step state machine along and hands
   back whatever it worked out last time. */
float readAltitude(float seaLevelPressure = SEA_LEVEL_PRESSURE);

/* Barometer temperature reading */
float readTemperature();

/* Barometer pressure reading */
float readPressure();

/*=========================== Internal functions ===========================*/

/* Read BMP085 calibration cofficients and write them in module */
void readBarometerCalibrationData();

/* Registry writter */
void writeRegister(uint8_t addr, uint8_t reg, uint8_t value);

/* Single registry reading */
uint8_t readOne(uint8_t addr, uint8_t reg);

/* Multiple registry reading */
void readMultiple(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *dest);

/* I2C read helper */
uint16_t read16u(uint8_t reg);

/* I2C read helper */
int16_t read16(uint8_t reg);

/* Read temperature from BMP085 based on calibration data*/
int32_t computeTrueTemperature(int32_t UT);

/* Read pressure from BMP085 based on calibration data*/
int32_t computeTruePressure(int32_t UP);

/* Convert pressure (in Pa) to the altitude above sea level*/
float pressureToAltitude(float pressure, float seaLevelPressure = SEA_LEVEL_PRESSURE);

/* Legacy raw BMP085 readings*/
int32_t readRawPressure();
int32_t readRawTemperature();

#endif
