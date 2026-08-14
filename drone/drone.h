#ifndef DRONE_H
#define DRONE_H

#include <Arduino.h>

/*
 * shared types
 *
 * Control_Data_t / State_t pair is the radio protocol and has to match
 * remote/remote.h - the structs go over the air without serialisation. 
 * Both boards share 8-bit AVR out of the same toolchain
 */

/*=========================== Radio ===========================*/

typedef enum {
  NONE_COMMAND = 0,
  DEPART,
  LAND,
  SHUTDOWN
} Command_t;

typedef enum {
  IDLE = 0,
  DEPARTING,
  FLYING,
  LANDING,
  SHOTDOWN
} Drone_State_t;

typedef struct {
  int8_t X;                     // -100..100
  int8_t Y;
} Joystick_t;

/* remote -> drone */
typedef struct {
  Joystick_t joystick;
  int8_t     gas;               // -100..100
  Command_t  command;
} Control_Data_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
} Angles_t;

/* drone -> remote */
typedef struct {
  Angles_t      angle;
  int8_t        gas;
  Drone_State_t current_command;
} State_t;

/*=========================== Flight controller ===========================*/

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
} PID_t;

typedef struct {
  float    altitude;            // metres above where we armed
  Angles_t angle;               // degrees
  uint16_t motors[4];           // us, FL FR BL BR
} Dynamics_State_t;

/*=========================== Loop timing ===========================*/

#define LOOP_PERIOD_US 10000UL      // 100 Hz
#define DT 0.01f

#define TELEMETRY_PERIOD_MS 100UL    // 10 Hz, same as the remote sends at

/* No packet for this long and we assume the remote is gone and land */
#define LINK_TIMEOUT_MS 1000UL

/* Complementary filter, gyro екгые кфешщ */
#define ALPHA 0.98f

/* Full joystick deflection = this tilt */
#define MAX_TILT_ANGLE 20.0f

#endif
