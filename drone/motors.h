#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>
#include "drone.h"

/*
 * 4 BLHeli_S ESCs on a normal servo pulse.
 * the arm-up sequence is still somewhat strange to me
 */
#define PPM_MIN      1000
#define PPM_MAX      1900

/* not sure? */
#define PPM_BENCH    1050

/* What is that? */
#define PPM_HOVER    1450

extern Servo motor_FL;
extern Servo motor_FR;
extern Servo motor_BL;
extern Servo motor_BR;

void setupMotors();

/* X-mixer. Writes the four ESCs and remembers what it wrote in state.motors */
void mixMotors(Dynamics_State_t &state, int baseThrottle,
               float rollOut, float pitchOut, float yawOut);

/* This arms up the motors */
void stopMotors(Dynamics_State_t &state);

#endif
