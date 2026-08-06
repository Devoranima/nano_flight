#include <SPI.h>
#include <RF24.h>

#include "radio.h"
#include "pins.h"

RF24 radio(CE_PIN, CSN_PIN);

/* RF24 addresses are 5 bytes long*/
uint8_t remote_address[5] = "TRSVR";
uint8_t drone_address[5] = "RECVR";

static unsigned long last_packet = 0;

void setupRadio() {
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening(drone_address);      // our TX address
  radio.openReadingPipe(1, remote_address);
  radio.enableDynamicPayloads();
  radio.startListening();
}

bool readControls(Control_Data_t &controls) {
  uint8_t pipe;
  if (radio.available(&pipe)) {
    radio.read(&controls, sizeof(controls));
    last_packet = millis();
    return true;
  }
  return false;
}

void sendState(const State_t &state) {
  radio.stopListening();
  radio.write(&state, sizeof(state));
  radio.startListening();
}

unsigned long lastPacketTime() {
  return last_packet;
}
