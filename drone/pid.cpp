#include "pid.h"

/* Cap on the integral, in ESC microseconds.*/
#define INTEGRAL_LIMIT 100.0f

float computePID(PID_t &pid, float error) {
  float p = pid.kp * error;

  pid.integral += error * DT;
  if (pid.integral > INTEGRAL_LIMIT)  pid.integral = INTEGRAL_LIMIT;
  if (pid.integral < -INTEGRAL_LIMIT) pid.integral = -INTEGRAL_LIMIT;
  float i = pid.ki * pid.integral;

  float d = pid.kd * (error - pid.previous_error) / DT;
  pid.previous_error = error;

  return p + i + d;
}

void resetPID(PID_t &pid) {
  pid.integral = 0.0f;
  pid.previous_error = 0.0f;
}
