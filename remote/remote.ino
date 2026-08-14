#include <SPI.h>
#include <RF24.h>

#include "pins.h"
#include "remote.h"

int last_encoder_state;

bool receiver_online = true;

RF24 radio(CE_PIN, CSN_PIN);

uint8_t remote_address[12] = "Transmitter";
uint8_t drone_address[9] = "Receiver";

Control_Data_t remote_controls;
State_t drone_state;

void setupRadio(){
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening(remote_address);
  radio.openReadingPipe(1, drone_address);
  radio.enableDynamicPayloads();
  radio.startListening();
}

void setupEncoder(){
  remote_controls.gas = 0;
  pinMode(CLK_PIN,INPUT_PULLUP);
  pinMode(DT_PIN,INPUT_PULLUP);
  last_encoder_state = digitalRead(CLK_PIN);
  // attachInterrupt(digitalPinToInterrupt(CLK_PIN), readEncoder, CHANGE);
}

void setupJoystick(){
  pinMode(VRX_PIN, INPUT);
  pinMode(VRY_PIN, INPUT);
}

void setupButtons(){
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
}

void setup() {
  Serial.begin(115200);

  setupRadio();
  setupEncoder();
  setupJoystick();
  setupButtons();

}  // setup

void readEncoder(){
  int encoder_state = digitalRead(CLK_PIN);
  if (encoder_state != last_encoder_state){
     if (digitalRead(DT_PIN) != encoder_state) { 
       remote_controls.gas ++;
     }
     else {
       remote_controls.gas --;
     }
    remote_controls.gas = constrain(remote_controls.gas, -100, 100);
  } 
  last_encoder_state = encoder_state;
}

void readJoystick(){
  int x_raw = analogRead(VRX_PIN);
  int y_raw = analogRead(VRY_PIN);

  remote_controls.joystick.X = -map(x_raw, 0, 1024, -100, 100);
  remote_controls.joystick.Y = -map(y_raw, 0, 1024, -100, 100);
}

void readButtons(){
  bool is_up_pressed = digitalRead(UP_PIN) == LOW;
  bool is_down_pressed = digitalRead(DOWN_PIN) == LOW;
  if (is_up_pressed){
    remote_controls.command = DEPART;
  }
  else if (is_down_pressed){
    remote_controls.command = LAND;
  }
  else {
    remote_controls.command = NONE_COMMAND;
  }
}

void sendControlsToDrone(){
  radio.stopListening();
  bool report = radio.write(&remote_controls, sizeof(remote_controls));  // transmit & save the report
  if (report) {
    receiver_online = true;
  }
  else{
    if (receiver_online){
      // Serial.println("Receiver dead");
      receiver_online = false;
    }
  }
  radio.startListening();
}

void readDroneState(){
  uint8_t pipe;
  if (radio.available(&pipe)){
    radio.read(&drone_state, sizeof(drone_state));
    printDroneState();
  }
}

void printDroneState(){
  // Serial.print(F("Pitch: "));Serial.print(drone_state.angle.pitch);
  // Serial.print(F(" Roll: "));Serial.print(drone_state.angle.roll);
  // Serial.print(F(" Yaw: "));Serial.print(drone_state.angle.yaw);
  Serial.print(F(" | Gas: "));Serial.print(drone_state.gas);
  Serial.print(F(" | Current Command:  "));
  switch (drone_state.current_command){
    case IDLE:
      Serial.println("Idle");
      break;
    case DEPARTING:
      Serial.println("Departing");
      break;
    case FLYING:
      Serial.println("Landing");
      break;
    case LANDING:
      Serial.println("Landing");
      break;
    case EMERGENCY_STOP:
      Serial.println("EMERGENCY_STOP");
      break;
  }
}

void printRemoteControls(){
  Serial.print(F("Gas: "));
  Serial.print(remote_controls.gas);
  Serial.print(F(" | X: "));
  Serial.print(remote_controls.joystick.X);
  Serial.print(F(" | Y: "));
  Serial.print(remote_controls.joystick.Y);
  Serial.print(F(" | Special command?: "));
  switch (remote_controls.command){
    case NONE_COMMAND:
      Serial.println("none");
      break;
    case DEPART:
      Serial.println("Depart");
      break;
    case LAND:
      Serial.println("Land");
      break;
    case SHUTDOWN:
      Serial.println("Shutdown");
      break;

  }
}

void loop() {
  readEncoder();
  readJoystick();
  readButtons();
  sendControlsToDrone();
  readDroneState();
  delay(100);
}  // loop
