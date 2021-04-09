#ifndef _SRUAV_CONTROL_H
#define _SRUAV_CONTROL_H

#include "config.h"
#include <BasicLinearAlgebra.h>
#include <Servo.h>
#include "dshot.h"

#define SERVO_MIN_TIMING 900
#define SERVO_MAX_TIMING 2100

#define MOTOR_MIN_DSHOT 0
#define MOTOR_MAX_DSHOT 1500 // No more is needed to lift aircraft.

using namespace BLA;

class Control
{
public:

    Control();

    void init();

    void set_motor( uint8_t index, uint16_t throttle );

    void set_servo( uint8_t index, float angle );

    void control_attitude( float roll, float pitch, float yaw, float gx, float gy, float gz );

    void get_rc_signals( void );

    uint16_t get_sp_throttle( void );

private:

    Servo* servos;
    

    static const uint16_t servo_pins[];
    static const uint16_t receiver_pins[];

    // LQR optimal gain for attitude controller
    Matrix<4,6> K = {   0.3527,    0.0000,    0.2491,    0.7023,    0.0000,    0.3359,
                       -0.0000,    0.3527,   -0.2491,    0.0000,    0.7023,   -0.3359,
                        0.3527,   -0.0000,   -0.2491,    0.7023,   -0.0000,   -0.3359,
                       -0.0000,    0.3527,    0.2491,   -0.0000,    0.7023,    0.3359};

    // State vector for attitude; roll, pitch, yaw, gx, gy, gz
    Matrix<6,1> X = {0,0,0,0,0,0};

    // Actuation vector / output
    Matrix<4,1> U = {0,0,0,0};

    // Setpoints for attitude controller
    Matrix<6,1> SP_rot = {0,0,0,0,0,0};

    // Throttle setpoint (controlled via RC controller)
    uint16_t SP_throttle = 0;

};


#endif