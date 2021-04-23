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

// Maps the kRPM output of the controller to DSHOT values. This varies with voltage, thus should probably implement RPM controller at some point
#define MOTOR_KRPM_TO_DSHOT 72.43f 
#define CONTROL_LOOP_INTERVAL 0.005f

using namespace BLA;

typedef struct __attribute__ ((packed)){
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

    void control_hover( float roll, float pitch, float yaw, float gx, float gy, float gz, float z, float vz );

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
    Only attitude controller
    Matrix<4,6> K = {   23.0746,    0.0000,   16.1308,    5.8192,    0.0000,    2.8430,
                         0.0000,   23.0746,  -16.1308,   -0.0000,    5.8195,   -2.8430,
                        23.0746,    0.0000,  -16.1308,    5.8192,    0.0000,   -2.8430,
                         0.0000,   23.0746,   16.1308,    0.0000,    5.8195,    2.8430}; */

    // Attitude and altitude controller. With integral action on altitude error.
    Matrix<5,9> K = {   22.3607,   -0.0000,   15.8114,    8.9976,   -0.0000,    5.6572,   -0.0000,   -0.0000,   -0.0000,
                        -0.0000,  -22.3607,  -15.8114,   -0.0000,   -8.9978,   -5.6572,   -0.0000,   -0.0000,   -0.0000,
                        22.3607,    0.0000,  -15.8114,    8.9976,    0.0000,   -5.6572,    0.0000,    0.0000,    0.0000,
                         0.0000,  -22.3607,   15.8114,    0.0000,   -8.9978,    5.6572,    0.0000,   -0.0000,   -0.0000,
                        -0.0000,   -0.0000,    0.0000,   -0.0000,   -0.0000,    0.0000,    3.7895,    2.9874,    2.2361 };
                        // roll    // pitch   // yaw     // gx      // gy       // gz      // z     // vz      // zint

    // State vector roll, pitch, yaw, gx, gy, gz, z, vz, zi
    Matrix<9,1> X = {0,0,0,0,0,0,0,0,0};

    // Actuation vector / output
    Matrix<5,1> U = {0,0,0,0,0};

    // Setpoints for attitude controller (roll, pitch, yaw, gx, gy, gz, z, vz)
    Matrix<8,1> SP = {0,0,0,0,0,0,1,0};

    // Throttle setpoint (controlled via RC controller)
    uint16_t SP_throttle = 0;


    int16_t servo_offset[4] = {0,0,0,0};

};


#endif