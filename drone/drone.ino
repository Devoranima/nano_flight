/*
 * Arduino Nano based flight controller
 *
 * my_sensors.* - I2C driver for the 9DOF board and the barometer
 * drone.ino - complementary filter, flight states, telemetry
 * pid.* - one regulator per axis
 * motors.* - X mixer, ESC output
 * radio.* - nRF24 link
 *
 * 100 Hz loop. 
 */

#include <Wire.h>

#include "drone.h"
#include "pins.h"
#include "my_sensors.h"
#include "pid.h"
#include "motors.h"
#include "radio.h"

/*! PID calibration values were lost completely, but some were saved in the thesis*/

/* PID calibration values for roll and pitch are the same, copying from my thesis*/
PID_t pid_roll     = {9.0, 0.03, 0.3, 0.0, 0.0};
PID_t pid_pitch    = {9.0, 0.03, 0.3, 0.0, 0.0};

/* Yaw and altitude are empty as I was unable to fint them */
PID_t pid_yaw      = {0.0, 0.0, 0.0, 0.0, 0.0};
PID_t pid_altitude = {0.0, 0.0, 0.0, 0.0, 0.0};

Dynamics_State_t current_dynamics = {0, {0, 0, 0}, {0, 0, 0, 0}};
Dynamics_State_t target_dynamics  = {0, {0, 0, 0}, {0, 0, 0, 0}};

Control_Data_t remote_controls = {{0, 0}, 0, NONE_COMMAND};
Drone_State_t  flight_state = IDLE;

unsigned long lastTime = 0;
unsigned long microsPrevious = 0;
unsigned long telemetryPrevious = 0;
unsigned long stateEntered = 0;

/* filter output, radians */
float roll = 0, pitch = 0, yaw = 0;

/* ground level, taken when the motors spin up */
float baseAltitude = 0;

int baseThrottle = PPM_MIN;
int landingFrom = PPM_MIN;   // throttle we were at when LANDING started


/*=========================== Sensors ===========================*/

void readSensors() {
  float ax, ay, az;
  float gx, gy, gz;
  readAccel(ax, ay, az);
  readGyro(gx, gy, gz);          // deg/s

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // angles straight off the gravity vector
  float accel_roll  = atan2(ay, az);
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));

  // gyro is the other way round: fine over a few ms, drifts over minutes
  roll  += gx * 0.0174533 * dt;
  pitch += gy * 0.0174533 * dt;
  yaw   += gz * 0.0174533 * dt;

  roll  = ALPHA * roll  + (1 - ALPHA) * accel_roll;
  pitch = ALPHA * pitch + (1 - ALPHA) * accel_pitch;
  // yaw gets no correction, there is nothing to correct it with

  float roll_deg  = roll  * 180 / PI;
  float pitch_deg = pitch * 180 / PI;
  float yaw_deg   = yaw   * 180 / PI;

  // board is mounted turned 90 degrees, so swap the axes back. The mixer
  // signs in motors.cpp depend on this.
  current_dynamics.angle.roll  = -pitch_deg;
  current_dynamics.angle.pitch = roll_deg;
  current_dynamics.angle.yaw   = yaw_deg;

  // returns the previous value immediately, does one step of the conversion
  current_dynamics.altitude = readAltitude(SEA_LEVEL_PRESSURE) - baseAltitude;
}


/*=========================== Control ===========================*/

void applyControls() {
  target_dynamics.angle.roll  = (remote_controls.joystick.X / 100.0) * MAX_TILT_ANGLE;
  target_dynamics.angle.pitch = (remote_controls.joystick.Y / 100.0) * MAX_TILT_ANGLE;
  // no yaw stick on the remote, target stays where it was
}

void updateMotors() {
  float errorRoll  = target_dynamics.angle.roll  - current_dynamics.angle.roll;
  float errorPitch = target_dynamics.angle.pitch - current_dynamics.angle.pitch;
  float errorYaw   = target_dynamics.angle.yaw   - current_dynamics.angle.yaw;

  float pidRollOut  = computePID(pid_roll, errorRoll);
  float pidPitchOut = computePID(pid_pitch, errorPitch);
  float pidYawOut   = computePID(pid_yaw, errorYaw);

  mixMotors(current_dynamics, baseThrottle, pidRollOut, pidPitchOut, pidYawOut);
}

void resetControllers() {
  resetPID(pid_roll);
  resetPID(pid_pitch);
  resetPID(pid_yaw);
  resetPID(pid_altitude);
}

void enterState(Drone_State_t next) {
  flight_state = next;
  stateEntered = millis();
}


/*=========================== Flight states ===========================*/
/* IDLE -> DEPARTING -> FLYING -> LANDING -> IDLE, plus SHOTDOWN as a kill
   switch. */

void updateFlightState() {
  bool link_lost = (millis() - lastPacketTime()) > LINK_TIMEOUT_MS;

  // kill switch beats everything else
  if (remote_controls.command == SHUTDOWN) {
    enterState(SHOTDOWN);
  }

  switch (flight_state) {
    case IDLE:
      baseThrottle = PPM_MIN;
      resetControllers();
      stopMotors(current_dynamics);
      if (remote_controls.command == DEPART && !link_lost) {
        baseAltitude = readAltitude(SEA_LEVEL_PRESSURE);
        yaw = 0;
        target_dynamics.angle.yaw = 0;
        enterState(DEPARTING);
      }
      break;

    case DEPARTING:
      // walk the throttle up to hover over two seconds, stabilising all the way
      baseThrottle = PPM_MIN + ((long)(PPM_HOVER - PPM_MIN) * (long)(millis() - stateEntered)) / 2000L;
      if (baseThrottle >= PPM_HOVER) {
        baseThrottle = PPM_HOVER;
        enterState(FLYING);
      }
      applyControls();
      updateMotors();
      if (link_lost || remote_controls.command == LAND) {
        landingFrom = baseThrottle;
        enterState(LANDING);
      }
      break;

    case FLYING:
      baseThrottle = constrain(PPM_HOVER + remote_controls.gas * 2, PPM_MIN, PPM_MAX);
      applyControls();
      updateMotors();
      if (link_lost || remote_controls.command == LAND) {
        landingFrom = baseThrottle;
        enterState(LANDING);
      }
      break;

    case LANDING:
      // bleed the throttle off over four seconds from wherever we were
      baseThrottle = landingFrom - ((long)(landingFrom - PPM_MIN) * (long)(millis() - stateEntered)) / 4000L;
      applyControls();
      updateMotors();
      if (baseThrottle <= PPM_MIN) enterState(IDLE);
      break;

    case SHOTDOWN:
      // stays dead until the board is power cycled
      stopMotors(current_dynamics);
      break;
  }
}


/*=========================== Telemetry ===========================*/

void sendTelemetry() {
  State_t state;
  state.angle = current_dynamics.angle;
  state.gas = remote_controls.gas;
  state.current_command = flight_state;
  sendState(state);
}


/*=========================== Serial tuning ===========================*/
/* "SET ROLL KP 900" -> kp = 9.00. Scaled by 100 so the parser never touches
   a float. Kept from the old bench sketch. */
void checkSerialTuning() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    if (input.startsWith("SET")) {
      char axis[6], gain[4];
      int val;
      int matched = sscanf(input.c_str(), "SET %5s %3s %d", axis, gain, &val);
      float value = (float)val;
      value = value / 100;

      if (matched == 3) {
        PID_t* pid = nullptr;
        if (strcmp(axis, "ROLL") == 0) pid = &pid_roll;
        else if (strcmp(axis, "PITCH") == 0) pid = &pid_pitch;
        else if (strcmp(axis, "YAW") == 0) pid = &pid_yaw;
        else if (strcmp(axis, "ALT") == 0) pid = &pid_altitude;

        if (pid) {
          if (strcmp(gain, "KP") == 0) pid->kp = value;
          else if (strcmp(gain, "KI") == 0) pid->ki = value;
          else if (strcmp(gain, "KD") == 0) pid->kd = value;
          Serial.print(axis); Serial.print(' ');
          Serial.print(gain); Serial.print(" = ");
          Serial.println(value);
        }
      }
    }
  }
}


/*=========================== setup / loop ===========================*/

void setup() {
  Serial.begin(115200);

  initializeSensors();
  setupRadio();
  setupMotors();

  // has to stand still for this - zeroes the gyro, loads the BMP085 constants
  calibrateSensors();
  baseAltitude = readAltitude(SEA_LEVEL_PRESSURE);

  lastTime = micros();
  microsPrevious = micros();
}

void loop() {
  checkSerialTuning();
  readControls(remote_controls);

  unsigned long microsNow = micros();
  if (microsNow - microsPrevious >= LOOP_PERIOD_US) {
    microsPrevious = microsNow;
    readSensors();
    updateFlightState();
  }

  unsigned long millisNow = millis();
  if (millisNow - telemetryPrevious >= TELEMETRY_PERIOD_MS) {
    telemetryPrevious = millisNow;
    sendTelemetry();
  }
}
