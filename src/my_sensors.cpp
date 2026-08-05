#include "my_sensors.h"

void initGyro(){
  // L3GD20 Gyro: enable XYZ, 95 Hz
  writeRegister(L3GD20_ADDRESS, 0x20, 0x00); // CTRL_REG1 = 0b00000000: clear reg
  writeRegister(L3GD20_ADDRESS, 0x20, 0x0F); // CTRL_REG1 = 0b00001111: normal mode, enable XYZ 
  writeRegister(L3GD20_ADDRESS, 0x23, 0x00); // CTRL_REG4 = 0b00000000: 250dps
}

void initAcc(){
  // LSM303 Accel: enable X, Y, Z, 100 Hz
  writeRegister(LSM303_ADDRESS_ACCEL, 0x20, 0x57); // CTRL_REG1_A = 0b01010111: 100Hz, normal mode, enable XYZ
}

void initMag(){
  // LSM303 Mag: 30Hz update rate, gain 1.3 Gauss
  writeRegister(LSM303_ADDRESS_MAG, 0x00, 0x14); // CRA_REG_M = 0b00010100: 30Hz 
  writeRegister(LSM303_ADDRESS_MAG, 0x01, 0x20); // CRB_REG_M = 0b00100000: gain ±1.3 Gauss
}

void initializeSensors() {
  Wire.begin();
  Wire.setClock(400000); // set I2C clock to 400 kHz

  initGyro();
  initAcc();

  // initMag() not used atm
}

double gx_cal, gy_cal, gz_cal;
int cal_int;

void calibrateSensors(){
#ifndef SILENT_OUTPUT
  Serial.print("Starting calibration...");
#endif
  float gx, gy, gz;
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){    //take 2000 readings for calibration
    readGyro(gx, gy, gz);                     //read the gyro output
    gx_cal += gx;                                    //ad roll value to gyro_roll_cal
    gy_cal += gy;                                    //ad pitch value to gyro_pitch_cal
    gz_cal += gz;                                    //ad yaw value to gyro_yaw_cal
    delay(4);                                        //wait 4 milliseconds before the next loop
#ifndef SILENT_OUTPUT
    if(cal_int%100 == 0)Serial.print(".");
#endif
  }
#ifndef SILENT_OUTPUT
    Serial.println(" done!");
#endif
  gx_cal /= 2000;
  gy_cal /= 2000;
  gz_cal /= 2000;

  readBarometerCalibrationData();
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readOne(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

void readMultiple(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *dest) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    dest[i] = Wire.read();
  }
}

uint16_t read16u(uint8_t reg) {
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP085_ADDRESS, (uint8_t)2);

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 8) | lsb;
}

int16_t read16(uint8_t reg) {
  return (int16_t)read16u(reg);
}

void readAccel(float &ax, float &ay, float &az) {
  uint8_t xlo = readOne(LSM303_ADDRESS_ACCEL, 0x28); // OUT_X_L_A
  uint8_t xhi = readOne(LSM303_ADDRESS_ACCEL, 0x29); // OUT_X_H_A
  uint8_t ylo = readOne(LSM303_ADDRESS_ACCEL, 0x2A); // OUT_Y_L_A
  uint8_t yhi = readOne(LSM303_ADDRESS_ACCEL, 0x2B); // OUT_Y_H_A
  uint8_t zlo = readOne(LSM303_ADDRESS_ACCEL, 0x2C); // OUT_Z_L_A
  uint8_t zhi = readOne(LSM303_ADDRESS_ACCEL, 0x2D); // OUT_Z_H_A

  int16_t rawX = (int16_t)(xlo | (xhi << 8));
  int16_t rawY = (int16_t)(ylo | (yhi << 8));
  int16_t rawZ = (int16_t)(zlo | (zhi << 8));

  // scale factor: 2g range, 12-bit mode (assuming CTRL_REG4_A = 0x00, as per datasheet)
  // 1 LSB = 1 mg, so multiply by 0.001 * g (9.8 m/s²) = 0.0098
  ax = rawX * 0.0098;
  ay = rawY * 0.0098;
  az = rawZ * 0.0098;
}

void readMag(float &mx, float &my, float &mz) {
  static int magCounter = 0;
  static float mx_last, my_last, mz_last;
  if (magCounter % 3 == 0) {
    uint8_t magBuffer[6];
    readMultiple(LSM303_ADDRESS_MAG, 0x03, 6, magBuffer);
    int16_t rawX = (int16_t)(magBuffer[0] << 8 | magBuffer[1]);
    int16_t rawY = (int16_t)(magBuffer[2] << 8 | magBuffer[3]);
    int16_t rawZ = (int16_t)(magBuffer[4] << 8 | magBuffer[5]);
    mx_last = rawX / 1100.0;
    my_last = rawY / 1100.0;
    mz_last = rawZ / 980.0;
  }
  magCounter++;
  mx = mx_last;
  my = my_last;
  mz = mz_last;
}

void readGyro(float &gx, float &gy, float &gz) {
  uint8_t gyroBuffer[6];
  readMultiple(L3GD20_ADDRESS, 0x28 | 0x80, 6, gyroBuffer);
  int16_t rawX = (int16_t)(gyroBuffer[1] << 8 | gyroBuffer[0]);
  int16_t rawY = (int16_t)(gyroBuffer[3] << 8 | gyroBuffer[2]);
  int16_t rawZ = (int16_t)(gyroBuffer[5] << 8 | gyroBuffer[4]);
  gx = rawX * 0.00875F;
  gy = rawY * 0.00875F;
  gz = rawZ * 0.00875F;
  // cal_int only hits 2000 after calibrateSensors() is done, which is what
  // keeps the offset out of the calibration samples.
  if (cal_int == 2000) {
    gx -= gx_cal;
    gy -= gy_cal;
    gz -= gz_cal;
  }
}

int16_t BMP085_CAL_AC1, BMP085_CAL_AC2, BMP085_CAL_AC3;
uint16_t BMP085_CAL_AC4, BMP085_CAL_AC5, BMP085_CAL_AC6;
int16_t BMP085_CAL_B1, BMP085_CAL_B2;
int16_t BMP085_CAL_MB, BMP085_CAL_MC, BMP085_CAL_MD;

void readBarometerCalibrationData() {
  BMP085_CAL_AC1 = read16(0xAA);
  BMP085_CAL_AC2 = read16(0xAC);
  BMP085_CAL_AC3 = read16(0xAE);
  BMP085_CAL_AC4 = read16u(0xB0);
  BMP085_CAL_AC5 = read16u(0xB2);
  BMP085_CAL_AC6 = read16u(0xB4);
  BMP085_CAL_B1  = read16(0xB6);
  BMP085_CAL_B2  = read16(0xB8);
  BMP085_CAL_MB  = read16(0xBA);
  BMP085_CAL_MC  = read16(0xBC);
  BMP085_CAL_MD  = read16(0xBE);
}


int32_t B5;

int32_t computeTrueTemperature(int32_t UT) {
  int32_t X1 = ((UT - BMP085_CAL_AC6) * BMP085_CAL_AC5) >> 15;
  int32_t X2 = ((int32_t)BMP085_CAL_MC << 11) / (X1 + BMP085_CAL_MD);
  B5 = X1 + X2;
  return (B5 + 8) >> 4; // in 0.1 °C
}

int32_t computeTruePressure(int32_t UP) {
  int32_t B6 = B5 - 4000;
  int32_t X1 = (BMP085_CAL_B2 * (B6 * B6 >> 12)) >> 11;
  int32_t X2 = (BMP085_CAL_AC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = (((((int32_t)BMP085_CAL_AC1) * 4 + X3) << OSS) + 2) >> 2;
  X1 = (BMP085_CAL_AC3 * B6) >> 13;
  X2 = (BMP085_CAL_B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = (BMP085_CAL_AC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> OSS);
  int32_t p = (B7 < 0x80000000) ? (B7 << 1) / B4 : (B7 / B4) << 1;
  X1 = (p >> 8) * (p >> 8);
  X1 = ( X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  return p + ((X1 + X2 + 3791) >> 4); // in Pa
}

float pressureToAltitude(float pressure, float seaLevelPressure) {
  return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

float readPressure() {
  int32_t UT = readRawTemperature();
  computeTrueTemperature(UT); // updates B5
  int32_t UP = readRawPressure();
  return computeTruePressure(UP);
}

float readTemperature() {
  int32_t UT = readRawTemperature();
  return computeTrueTemperature(UT) / 10.0; // convert to °C
}

float readAltitude(float seaLevelPressure) {
  static enum { IDLE_STATE, WAIT_TEMP, WAIT_PRESS } state = IDLE_STATE;
  static unsigned long startTime = 0;
  static float altitude = 0;
  static int32_t UT, UP;
  unsigned long currentTime = millis();

  switch (state) {
    case IDLE_STATE:
      if (currentTime - startTime >= 100) {
        Wire.beginTransmission(BMP085_ADDRESS);
        Wire.write(0xF4);
        Wire.write(0x2E); // start temperature conversion
        Wire.endTransmission();
        startTime = currentTime;
        state = WAIT_TEMP;
      }
      break;
    case WAIT_TEMP:
      if (currentTime - startTime >= 5) { // 5 ms conversion time
        UT = read16u(0xF6); // read temperature
        computeTrueTemperature(UT);
        Wire.beginTransmission(BMP085_ADDRESS);
        Wire.write(0xF4);
        Wire.write(0x34 + (OSS << 6)); // start pressure conversion
        Wire.endTransmission();
        startTime = currentTime;
        state = WAIT_PRESS;
      }
      break;
    case WAIT_PRESS:
      if (currentTime - startTime >= (2 + (3 << OSS))) { // 14 ms for OSS=2
        Wire.beginTransmission(BMP085_ADDRESS);
        Wire.write(0xF6);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)BMP085_ADDRESS, (uint8_t)3);
        int32_t msb = Wire.read();
        int32_t lsb = Wire.read();
        int32_t xlsb = Wire.read();
        UP = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - OSS);
        int32_t pressure = computeTruePressure(UP);
        altitude = pressureToAltitude(pressure, seaLevelPressure);
        state = IDLE_STATE;
      }
      break;
  }
  return altitude;
}


// legacy, not used in optimized readAltitude
int32_t readRawTemperature() {
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);
  return read16u(0xF6);
}

int32_t readRawPressure() {
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS << 6));
  Wire.endTransmission();
  delay(2 + (3 << OSS));
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP085_ADDRESS, (uint8_t)3);
  int32_t msb = Wire.read();
  int32_t lsb = Wire.read();
  int32_t xlsb = Wire.read();
  return ((msb << 16) + (lsb << 8) + xlsb) >> (8 - OSS);
}
