#ifndef _SRUAV_CONFIG_H
#define _SRUAV_CONFIG_H

#include <Arduino.h>
#include "stdint.h"

#define MOTOR_POLES 14    // Number of poles in connected motor
#define TLM_INTERVAL 5000  // Interval in us

//These pins can be any GPIO
#define FLOW_CS_PIN 6
#define IMU_CS_PIN 10
#define IMU_WAK_PIN 22
#define IMU_INT_PIN 21
#define IMU_RST_PIN 20

#define SERVO_1_PIN 2
#define SERVO_2_PIN 3
#define SERVO_3_PIN 4
#define SERVO_4_PIN 5

// Pins
/* uint16_t servo_pins[]       = {2,3,4,5};
uint16_t receiver_pins[]    = {18, 19, 23, 1};
uint16_t motor_pins[]       = {};
uint16_t telemetry_pins[]   = {}; */



#endif