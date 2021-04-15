#include "communication.h"

Communication::Communication( void  ){
}

void Communication::init(){
    
    Serial3.begin(921600); // HIGH Baudrate for fast transmission of LOTS of telemetry
	wifi_serial = &Serial3;
}

void Communication::send_tlm( sensor_data_t output, control_signal_t input ){

    memset(tx_buffer, 0, sizeof(tx_buffer)); 

    // Build telemetry string to send over WiFI

    sprintf( tx_buffer + strlen(tx_buffer), "%.2f,%.2f,%.2f,%.2f,%d,", input.a1, input.a2, input.a3, input.a4, input.dshot );
    sprintf( tx_buffer + strlen(tx_buffer), "%.2f,%.2f,%.2f,", output.ax, output.ay, output.az );
    sprintf( tx_buffer + strlen(tx_buffer), "%.2f,%.2f,%.2f,", output.gx, output.gy, output.gz );
    sprintf( tx_buffer + strlen(tx_buffer), "%.2f,%.2f,%.2f,", output.roll, output.pitch, output.yaw );
    sprintf( tx_buffer + strlen(tx_buffer), "%.2f,%.2f,%.2f,", output.vx, output.vy, output.vz );
    sprintf( tx_buffer + strlen(tx_buffer), "%.2f,%.2f,%.2f,", output.x, output.y, output.z );
    sprintf( tx_buffer + strlen(tx_buffer), "%.3f,%.3f,%.3f,%.3f,", output.qw, output.qi, output.qj, output.qk );

    wifi_serial->println( tx_buffer );


}