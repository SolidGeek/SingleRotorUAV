#include "logger.h"

Logger::Logger( void  ){
}

void Logger::init(){

    Serial3.begin(921600); // HIGH Baudrate for fast transmission of LOTS of telemetry
	wifi_serial = &Serial3;

    transfer = new SerialTransfer();
    transfer->begin(Serial3);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Card failed, or not present");
        sd_ready = false;
    }else
        sd_ready = true;

    
}

void Logger::write_esp( sensor_data_t measurements, control_signal_t controls ){

    memset(&tx_buffer, 0, sizeof(tx_buffer)); 
    tx_buffer.x = measurements;
    tx_buffer.u = controls;

    // Send struct of data over UART
    transfer->sendDatum( tx_buffer );
}

void Logger::start_log(){
    if( !is_logging ){
        is_logging = true;
        sprintf( log_name, "log_%d.dat", log_id );
    }
}

void Logger::end_log(){
    if( is_logging ){
        is_logging = false;
        log_id++;
    }
}

void Logger::write_sd( sensor_data_t measurements, control_signal_t controls  ){

    if( !is_logging || !sd_ready )
        return;

    memset(&tx_buffer, 0, sizeof(tx_buffer)); 
    tx_buffer.x = measurements;
    tx_buffer.u = controls;

    File log = SD.open( log_name, FILE_WRITE );

    log.write( (uint8_t *) &tx_buffer, sizeof(tx_buffer) );
    log.close();

}