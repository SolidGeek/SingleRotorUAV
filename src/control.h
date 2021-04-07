#ifndef _SRUAV_CONTROL_H
#define _SRUAV_CONTROL_H

#include "config.h"
#include <Servo.h>
#include "dshot.h"

#define SERVO_MIN_TIMING 900
#define SERVO_MAX_TIMING 2100

class Control
{
public:

    Control();

    void init();

    void control_motor( uint8_t index, float dshot );

    void control_servo( uint8_t index, float angle );

private:

    Servo* servos;

    static const uint16_t servo_pins[];

};


#endif