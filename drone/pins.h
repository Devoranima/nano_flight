#ifndef PINS_H
#define PINS_H

/*=========================== ESC / motors ===========================*/
#define MOTOR_FL_PIN    2
#define MOTOR_FR_PIN    3
#define MOTOR_BL_PIN    4
#define MOTOR_BR_PIN    5

/*=========================== nRF24L01 ===============================*/
#define CE_PIN          9
#define CSN_PIN         8

/* RF24 uses default pins, keep these just as a reminder*/
#define SCK_PIN         13
#define MOSI_PIN        11
#define MISO_PIN        12

/*=========================== Sensors ================================*/
/* 9DOF and BMP085 both sit on the I2C bus, SDA A4 / SCL A5 */

#endif
