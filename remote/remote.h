#ifndef REMOTE_H
#define REMOTE_H

#include <Arduino.h>

/*
 * Radio protocol, remote side. Same block as in drone/drone.h
 */

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
  EMERGENCY_STOP
} Drone_State_t;

typedef struct {
  int8_t X;                     // -100..100
  int8_t Y;
} Joystick_t;

/* remote -> drone */
typedef struct {
  Joystick_t joystick;
  int8_t     gas;
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

#endif
