#ifndef _SRUAV_COMMS_H
#define _SRUAV_COMMS_H

#include <Arduino.h>
#include "SerialTransfer.h" // Transfer and reconstruction of large data objects over serial
#include "sensors.h"
#include "control.h"

// Telemetry package
typedef struct __attribute__ ((packed)){
    sensor_data_t data;         // System output (states)
    estimator_data_t estimate;  // Estimator output 
    control_signal_t control;   // System input  (actuation)
} tlm_data_t;


typedef struct __attribute__ ((package)){
    uint8_t command;
    float value;
} command_t;


typedef enum{
    COMMAND_ARM        = 0,
    COMMAND_DISARM     = 1,
    COMMAND_TAKEOFF    = 2,
    COMMAND_LAND       = 3,
    COMMAND_SETPOINT_X = 4,
    COMMAND_SETPOINT_Y = 5,
    COMMAND_SETPOINT_Z = 6,
    COMMAND_SET_ORIGIN = 7,
    DATA_POSITION_X    = 10,
    DATA_POSITION_Y    = 11,
    DATA_POSITION_Z    = 12
} command_type_t; 


class Communication{

public:

    Communication( void );

    void init( void );

    // For sending telemetry: UART --> ESP --> WIFI
    void write_udp_telemetry( tlm_data_t package );

    // For receiving commands: UART <-- ESP <-- WIFI
    int read_udp_commands();

    float get_command_value();

private:

    Stream * serial_port;
    SerialTransfer * uart_transfer;

    tlm_data_t telemetry_buffer;
    command_t command_buffer;

};

#endif