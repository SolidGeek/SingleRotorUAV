#ifndef _SRUAV_CONTROL_H
#define _SRUAV_CONTROL_H

#include "config.h"
#include <BasicLinearAlgebra.h>
#include <Servo.h>
#include "dshot.h"

#define SERVO_MIN_TIMING 900
#define SERVO_MAX_TIMING 2100

using namespace BLA;

class Control
{
public:

    Control();

    void init();

    void control_motor( uint8_t index, float dshot );

    void control_servo( uint8_t index, float angle );

    void attitude( float roll, float pitch, float yaw, float gx, float gy, float gz );

private:

    Servo* servos;

    static const uint16_t servo_pins[];

    // LQR optimal gain for attitude controller
    Matrix<4,6> K = {    3.5073,   -0.0000,    0.4973,    2.2080,    0.0000,    0.4798,
                        -0.0000,    3.5073,   -0.4973,    0.0000,    2.2082,   -0.4798,
                         3.5073,   -0.0000,   -0.4973,    2.2080,   -0.0000,   -0.4798,
                        -0.0000,    3.5073,    0.4973,    0.0000,    2.2082,    0.4798};

    // State vector for attitude; roll, pitch, yaw, gx, gy, gz
    Matrix<6,1> X = {0,0,0,0,0,0};

    // Actuation vector / output
    Matrix<4,1> U = {0,0,0,0};

};


#endif