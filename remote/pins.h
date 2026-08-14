#ifndef PINS_H
#define PINS_H

/*
 * Remote pin map. Basically nothing survivded, had to restore by memory + ask claude for fixes 
 * nRF24 on SPI, two axis joystick, encoder for throttle, two buttons.
 *
 * The thesis is also a bit unstable (because of the rebuilds): chapter 5 opens
 * with "Arduino Micro" and then describes everything as wired to an Arduino
 * Nano. D2/D3 are the interrupt pins on both, so the encoder works either
 * way
 */

/*=========================== nRF24L01 ===============================*/
#define CE_PIN      9
#define CSN_PIN     8

/*=========================== Joystick ===============================*/
#define VRX_PIN     A0
#define VRY_PIN     A1

/*=========================== Encoder ================================*/
/* interrupt capable, so readEncoder() can move onto an ISR later - see the
   commented out attachInterrupt in setupEncoder() */
#define CLK_PIN     2
#define DT_PIN      3

/*=========================== Buttons ================================*/
#define UP_PIN      5     // DEPART
#define DOWN_PIN    6     // LAND

#endif
