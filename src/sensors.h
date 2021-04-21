#ifndef _SRUAV_SENSORS_H
#define _SRUAV_SENSORS_H

#include "constants.h"
#include "math.h"
#include <Wire.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
#include "BNO080.h"     // IMU 
#include "PMW3901.h"    // FLOW
#include "VL53L1X.h"

using namespace BLA;

#define STATUS_READY 0x01
#define STATUS_FAILED_SETUP 0x02
#define STATUS_NO_RESPONSE 0x03

typedef struct __attribute__ ((packed)){
    float gx, gy, gz;
    float roll, pitch, yaw;
    float qw, qi, qj, qk;
    float ax, ay, az;
    float vx, vy, vz;
    float x, y, z;
    struct{ // Bitfield, using 1 byte, to represent if new measurements are available
        uint8_t imu     : 1;
        uint8_t flow    : 1;
        uint8_t lidar   : 1;
    } status;
} sensor_data_t;

typedef struct{
    uint8_t imu;
    uint8_t flow;
    uint8_t lidar;
} sensor_status_t;


class Sensors
{
public:

    Sensors( void );
    
    // Init the sensor objects
    void init( void );

    void sample_imu();

    void sample_lidar();

    void sample_flow();

    // If interpolated measurements are needed, maybe Kalman.
    void run_estimator();

    void rotate_to_world( float * vector );

    sensor_data_t get_samples();

    sensor_data_t data;
    sensor_status_t status;

private:

    // Placeholders for sensor objects
    BNO080 * imu;
    PMW3901 * flow; 
    VL53L1X * lidar; 

    // Low pass filter to filter sensor data
    float LPF( float new_sample, float old_sample, float alpha );

    uint32_t last_flow_sample;

};


#endif