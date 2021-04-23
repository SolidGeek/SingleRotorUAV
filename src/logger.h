#ifndef _SRUAV_LOGGER_H
#define _SRUAV_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "SerialTransfer.h" // Transfer and reconstruction of large data objects over serial
#include "sensors.h"
#include "control.h"

typedef struct __attribute__ ((packed)){
    sensor_data_t data;    // System output (states)
    estimator_data_t estimate;
    control_signal_t control; // System input  (actuation)
} tlm_data_t;

class Logger
{
public:

    Logger( void );

    void init( void );

    // For sending telemetry over UART -> ESP -> WIFI
    void write_esp( tlm_data_t package );

    // For simple logging on SD card
    // void write_sd( sensor_data_t measurements, control_signal_t controls );

    // Starting and ending SD log. Each time called, a new log is created
    void start_log();

    void end_log();

private:

    Stream * wifi_serial;
    SerialTransfer * transfer;

    // char tx_buffer[500]; 

    tlm_data_t tx_buffer; 

    uint8_t log_id = 0;
    bool is_logging = false;
    bool sd_ready = false;

    char log_name[20]; 

};

#endif