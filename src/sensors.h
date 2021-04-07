#ifndef _SRUAV_SENSORS_H
#define _SRUAV_SENSORS_H

#include "config.h"
#include <Wire.h>
#include <SPI.h>
#include "BNO080.h"     // IMU 
#include "PMW3901.h"    // FLOW
#include "VL53L1X.h"


#define STATUS_READY 0x01
#define STATUS_FAILED_SETUP 0x02
#define STATUS_NO_RESPONSE 0x03

class Sensors
{
public:

    struct sensor_data
    {
        float gx, gy, gz;
        float ax, ay, az;
        float roll, pitch, yaw;
        float vx, vy, vz;
        float x, y, z;
    } data;

    Sensors( void );
    
    // Init the sensor objects
    void init( void );

    void sample_imu();

    void sample_lidar();

    void sample_flow();

    // If interpolated measurements are needed, maybe Kalman.
    void run_estimator();

    struct sensor_status{
        uint8_t imu;
        uint8_t flow;
        uint8_t lidar;
    } status;

private:

    // Placeholders for sensor objects
    BNO080 * imu;
    PMW3901 * flow; 
    VL53L1X * lidar; 

    

};


#endif