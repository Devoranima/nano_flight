#ifndef RADIO_H
#define RADIO_H

#include "drone.h"

/* Controls come in at about 10 Hz, telemetry goes back on the same tick. */
void setupRadio();

/* Fills controls and returns true if a packet arrived since last call */
bool readControls(Control_Data_t &controls);

void sendState(const State_t &state);

/* millis() of the last packet we heard, for the link timeout */
unsigned long lastPacketTime();

#endif
