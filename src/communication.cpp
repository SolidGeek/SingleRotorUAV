#include "communication.h"

Communication::Communication( void  ){
}

void Communication::init(){

    Serial3.begin(921600); // HIGH Baudrate for fast transmission of LOTS of telemetry
	serial_port = &Serial3;

    uart_transfer = new SerialTransfer();
    uart_transfer->begin(*serial_port);
    
}

void Communication::write_udp_telemetry( tlm_data_t package ){

    memset(&telemetry_buffer, 0, sizeof(telemetry_buffer)); 
    telemetry_buffer = package;
    package.timestamp = micros();

    // Send struct of data over UART
    uart_transfer->sendDatum( telemetry_buffer );
}

int Communication::read_udp_commands(){
    if( uart_transfer->available()  ){

        // Read struct into command_buffer
        uart_transfer->rxObj( command_buffer );

        return command_buffer.command;
    }

    return -1;
}

float * Communication::get_command_values(){
    return command_buffer.value;
}