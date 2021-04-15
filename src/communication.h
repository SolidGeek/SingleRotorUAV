#ifndef _SRUAV_COMM_H
#define _SRUAV_COMM_H

#include <Arduino.h>
#include "sensors.h"
#include "control.h"

class Communication
{
public:

    Communication( void );

    void init( void );

    void send_tlm( sensor_data_t output, control_signal_t input );


private:

    Stream * wifi_serial;

    char tx_buffer[500]; 

};

#endif