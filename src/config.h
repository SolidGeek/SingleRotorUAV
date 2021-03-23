#ifndef _SRUAV_CONFIG_H
#define _SRUAV_CONFIG_H

#include <Arduino.h>
#include "stdint.h"

#define MOTOR_POLES 14    // Number of poles in connected motor
#define TLM_INTERVAL 5000  // Interval in us

//These pins can be any GPIO
#define IMU_CS_PIN 10
#define IMU_WAK_PIN 22
#define IMU_INT_PIN 21
#define IMU_RST_PIN 20

uint16_t servo_pins[] = {2,3,4,5};
uint16_t receiver_pins[] = {18, 19, 23, 1};





#endif