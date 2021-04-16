#ifndef _SRUAV_CONTROL_H
#define _SRUAV_CONTROL_H

#include "constants.h"
#include "dshot.h"
#include <BasicLinearAlgebra.h>
#include <Servo.h>


#define SERVO_MIN_TIMING 900
#define SERVO_MID_TIMING 1500
#define SERVO_MAX_TIMING 2100

#define MOTOR_MIN_DSHOT 0
#define MOTOR_MAX_DSHOT 1500 // No more is needed to lift aircraft.

using namespace BLA;

typedef struct
{
    float a1, a2, a3, a4;   // Servo angles
    uint16_t dshot;            // Motor signal
} control_signal_t;

class Control
{
public:

    Control();

    void init();

    void write_motor( uint8_t index, uint16_t throttle );

    void write_servo( uint8_t index, float angle );

    void write_servo_ms( uint8_t index, uint16_t ms );

    void control_attitude( float roll, float pitch, float yaw, float gx, float gy, float gz );

    void get_rc_signals( void );

    uint16_t get_sp_throttle( void );

    void servo_calibration( int16_t * servo_offset );

    void set_servo_offsets( int16_t * servo_offset );
    
    // Used for telemetry only
    control_signal_t data; 

private:

    Servo* servos;
    

    static const uint16_t servo_pins[];
    static const uint16_t receiver_pins[];

    // LQR optimal gain for attitude controller
    /* 
    Matrix<4,6> K = {   13.9069,    0.0000,    9.7404,    4.6030,    0.0000,    2.2943,
                         0.0000,   13.9069,   -9.7404,    0.0000,    4.6032,   -2.2943,
                        13.9069,    0.0000,   -9.7404,    4.6030,    0.0000,   -2.2943,
                         0.0000,   13.9069,    9.7404,    0.0000,    4.6032,    2.2943}; */

    Matrix<4,6> K = {   23.0746,    0.0000,   16.1308,    5.8192,    0.0000,    2.8430,
                         0.0000,   23.0746,  -16.1308,   -0.0000,    5.8195,   -2.8430,
                        23.0746,    0.0000,  -16.1308,    5.8192,    0.0000,   -2.8430,
                         0.0000,   23.0746,   16.1308,    0.0000,    5.8195,    2.8430};

    // State vector for attitude; roll, pitch, yaw, gx, gy, gz
    Matrix<6,1> X = {0,0,0,0,0,0};

    // Actuation vector / output
    Matrix<4,1> U = {0,0,0,0};

    // Setpoints for attitude controller
    Matrix<6,1> SP_rot = {0,0,0,0,0,0};

    // Throttle setpoint (controlled via RC controller)
    uint16_t SP_throttle = 0;


    int16_t servo_offset[4] = {0,0,0,0};

};


#endif