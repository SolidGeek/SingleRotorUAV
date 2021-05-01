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

#define DT 0.005f
#define SENSOR_LIDAR_OFFSET 0.08f

typedef struct __attribute__ ((packed)){
    float gx, gy, gz;
    float roll, pitch, yaw;
    float ax, ay, az;
    float vx, vy;
    float z;
    struct{ // Bitfield, using 1 byte, to represent if new measurements are available
        uint8_t imu     : 1;
        uint8_t flow    : 1;
        uint8_t lidar   : 1;
    } status;
} sensor_data_t; // 49 bytes

typedef struct __attribute__ ((packed)){
    float x, y, z;
    float vx, vy, vz;
} estimator_data_t; // 24 bytes

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

    void set_origin();

    // If interpolated measurements are needed, maybe Kalman.
    void run_estimator();

    void update_pos_x ( float x );
    void update_pos_y ( float y );

    sensor_data_t get_samples();

    sensor_data_t data;
    estimator_data_t estimate;

    sensor_status_t status;

private:

    // Placeholders for sensor objects
    BNO080 * imu;
    PMW3901 * flow; 
    VL53L1X * lidar; 


    float yaw_origin = 0;
    float yaw_raw = 0;

    float vicon_pos_x = 0;
    float vicon_pos_y = 0;

    bool stat_pos_x = false;
    bool stat_pos_y = false;

    // Low pass filter to filter sensor data
    float LPF( float new_sample, float old_sample, float alpha );

    void rotate_to_world( float * vector );

    float rotate_yaw( float yaw );

    uint32_t last_flow_sample;



    // Estimator matrixes
    Matrix<6,6> A = {   1,  0,  0,  DT, 0,  0,
                        0,  1,  0,  0,  DT, 0,
                        0,  0,  1,  0,  0,  DT,
                        0,  0,  0,  1,  0,  0,
                        0,  0,  0,  0,  1,  0, 
                        0,  0,  0,  0,  0,  1 };

    Matrix<6,3> B = {   0.5*pow(DT,2),  0,              0,         
                        0,              0.5*pow(DT,2),  0,         
                        0,              0,              0.5*pow(DT,2),  
                        DT,             0,              0,         
                        0,              DT,             0,         
                        0,              0,              DT };
 
    Matrix<6,6> H; 

    // State vector
    Matrix<6,1> X = {0,0,0,0,0,0};

    // Prediction vector
    Matrix<6,1> Xpre;

    // Measurement vector
    Matrix<6,1> Z = {0,0,0,0,0,0};

    // Input vector
    Matrix<3,1> U = {0,0,0};

    // Estimator gain
    /* Matrix<6,6> Kf = {  0.0001,    0.0000,    0.0000,    0.0095,   -0.0000,    0.0000,
                        0.0000,    0.0001,   -0.0000,    0.0000,    0.0095,   -0.0000,
                        0.0000,   -0.0000,    0.1547,    0.0000,   -0.0000,    0.0000,
                        0.0000,    0.0000,    0.0000,    0.1057,   -0.0000,    0.0000,
                       -0.0000,    0.0000,   -0.0000,   -0.0000,    0.1057,   -0.0000,
                        0.0000,   -0.0000,    1.3001,    0.0000,   -0.0000,    0.0000  }; */

    /* Matrix<6,6> Kf = {  0.0345,    0.0000,   0.0000,    0.0019,    0.0000,    0.0000,
                        0.0000,    0.0345,   0.0000,    0.0000,    0.0019,    0.0000,
                        0.0000,    0.0000,   0.0274,    0.0000,    0.0000,    0.0001,
                        0.1495,    0.0000,   0.0000,    0.0216,    0.0000,    0.0000,
                        0.0000,    0.1495,   0.0000,    0.0000,    0.0216,    0.0000,
                        0.0000,    0.0000,   0.0767,    0.0000,    0.0000,    0.0004    }; */

    Matrix<6,6> Kf = {   0.2023,    0.0000,    0.0000,    0.0010,    0.0000,    0.0000,
                         0.0000,    0.2023,    0.0000,    0.0000,    0.0010,    0.0000,
                        -0.0000,    0.0000,    0.2449,    0.0000,    0.0000,    0.0000,
                         5.1657,    0.0000,    0.0000,    0.0667,    0.0000,    0.0000,
                         0.0000,    5.1657,    0.0000,    0.0000,    0.0667,    0.0000,
                        -0.0000,    0.0000,    6.8685,    0.0000,    0.0000,    0.0000 };

};


#endif