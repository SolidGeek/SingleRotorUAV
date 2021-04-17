#ifndef _SRUAV_COMM_H
#define _SRUAV_COMM_H

#include <Arduino.h>
#include "SerialTransfer.h" // Transfer and reconstruction of large data objects over serial
#include "sensors.h"
#include "control.h"


typedef struct __attribute__ ((packed)){
    sensor_data_t x;    // System output (states)
    control_signal_t u; // System input  (actuation)
} tlm_data_t;

class Communication
{
public:

    Communication( void );

    void init( void );

    void send_tlm( sensor_data_t output, control_signal_t input );


private:

    Stream * wifi_serial;
    SerialTransfer * transfer;

    // char tx_buffer[500]; 

    tlm_data_t tx_buffer; 

};

#endif