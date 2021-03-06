#ifndef _SRUAV_COMMS_H
#define _SRUAV_COMMS_H

#include <Arduino.h>
#include "SerialTransfer.h" // Transfer and reconstruction of large data objects over serial
#include "sensors.h"
#include "control.h"

// Telemetry package
typedef struct __attribute__ ((packed)){
    uint32_t timestamp;
    uint32_t index; 
    sensor_data_t data;         // System output (states)
    estimator_data_t estimate;  // Estimator output 
    control_signal_t control;   // System input  (actuation)
} tlm_data_t;

typedef struct __attribute__ ((package)){
    uint8_t command;
    float value[10];
} command_t;

typedef enum{
    COMMAND_ARM        = 0,
    COMMAND_DISARM     = 1,
    COMMAND_TAKEOFF    = 2,
    COMMAND_LAND       = 3,
    COMMAND_SET_X      = 4,
    COMMAND_SET_Y      = 5,
    COMMAND_SET_Z      = 6,
    COMMAND_SET_ROLL   = 7,
    COMMAND_SET_PITCH  = 8,
    COMMAND_SET_YAW    = 9,
    COMMAND_SET_ORIGIN = 10,
    COMMAND_LOITER     = 11,
    DATA_UPDATE_POS    = 20
} command_type_t; 

class Communication{

public:

    Communication( void );

    void init( void );

    // For sending telemetry: UART --> ESP --> WIFI
    void write_udp_telemetry( tlm_data_t package );

    // For receiving commands: UART <-- ESP <-- WIFI
    int read_udp_commands();

    float * get_command_values();

private:

    Stream * serial_port;
    SerialTransfer * uart_transfer;

    tlm_data_t telemetry_buffer;
    command_t command_buffer;

};

#endif