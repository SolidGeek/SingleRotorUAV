#ifndef _SRUAV_CONFIG_H
#define _SRUAV_CONFIG_H

#include <Arduino.h>
#include "stdint.h"


//These pins can be any GPIO
uint8_t imu_cs_pin = 10;
uint8_t imu_wak_pin = 22;
uint8_t imu_int_pin = 21;
uint8_t imu_rst_pin = 20;

uint16_t servo_pins[] = {2,3,4,5};
uint16_t receiver_pins[] = {18, 19, 23, 1};





#endif