#ifndef PID_H
#define PID_H

#include "drone.h"

/*
 * u = kp*e + ki*sum(e)*dt + kd*(e - e_prev)/dt
 *
 * One of these per axis. dt is fixed at the loop period for easier usage
 */

float computePID(PID_t &pid, float error);

/* Clear the integral and the last error. Call on motors off */
void resetPID(PID_t &pid);

#endif
