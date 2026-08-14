#include "motors.h"
#include "pins.h"

Servo motor_FL;
Servo motor_FR;
Servo motor_BL;
Servo motor_BR;

void setupMotors() {
  motor_FL.attach(MOTOR_FL_PIN);
  motor_FR.attach(MOTOR_FR_PIN);
  motor_BL.attach(MOTOR_BL_PIN);
  motor_BR.attach(MOTOR_BR_PIN);

  // BLHeli_S wants a low signal for a moment before it arms
  motor_FL.writeMicroseconds(PPM_MIN);
  motor_FR.writeMicroseconds(PPM_MIN);
  motor_BL.writeMicroseconds(PPM_MIN);
  motor_BR.writeMicroseconds(PPM_MIN);
  delay(3000);
}

void mixMotors(Dynamics_State_t &state, int baseThrottle,
               float rollOut, float pitchOut, float yawOut) {
  int m1 = baseThrottle + pitchOut - rollOut + yawOut;   // FL
  int m2 = baseThrottle + pitchOut + rollOut - yawOut;   // FR
  int m3 = baseThrottle - pitchOut - rollOut - yawOut;   // BL
  int m4 = baseThrottle - pitchOut + rollOut + yawOut;   // BR

  m1 = constrain(m1, PPM_MIN, PPM_MAX);
  m2 = constrain(m2, PPM_MIN, PPM_MAX);
  m3 = constrain(m3, PPM_MIN, PPM_MAX);
  m4 = constrain(m4, PPM_MIN, PPM_MAX);

  motor_FL.writeMicroseconds(m1);
  motor_FR.writeMicroseconds(m2);
  motor_BL.writeMicroseconds(m3);
  motor_BR.writeMicroseconds(m4);

  state.motors[0] = m1;
  state.motors[1] = m2;
  state.motors[2] = m3;
  state.motors[3] = m4;
}

void stopMotors(Dynamics_State_t &state) {
  motor_FL.writeMicroseconds(PPM_MIN);
  motor_FR.writeMicroseconds(PPM_MIN);
  motor_BL.writeMicroseconds(PPM_MIN);
  motor_BR.writeMicroseconds(PPM_MIN);

  for (uint8_t i = 0; i < 4; i++) state.motors[i] = PPM_MIN;
}
