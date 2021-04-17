// Load Wi-Fi library
#include <WiFi.h>
#include "SerialTransfer.h"

typedef struct __attribute__ ((packed)){
  float gx, gy, gz;
  float roll, pitch, yaw;
  float qw, qi, qj, qk;
  float ax, ay, az;
  float vx, vy, vz;
  float x, y, z;
} sensor_data_t;

typedef struct __attribute__ ((packed)){
  float a1, a2, a3, a4;   // Servo angles
  uint16_t dshot;         // Motor signal
} control_signal_t;

// Serial transfer object
typedef struct __attribute__ ((packed)){
  sensor_data_t x;    // System output (states)
  control_signal_t u; // System input  (actuation)
} tlm_data_t;

tlm_data_t rx_buffer; 
const size_t tx_size = sizeof(tlm_data_t)+3;
uint8_t tx_buffer[tx_size];

int led_pin = 12;

// Replace with your network credentials
const char* ssid     = "SR-UAV GUI";
const char* password = "123456789";

const char * UDP_address = "255.255.255.255";
const int UDP_port = 8080;

//The udp library class
WiFiUDP UDP;
SerialTransfer UART;

void setup() {
  Serial.begin(921600);
  UART.begin(Serial);
  
  pinMode(led_pin, OUTPUT);

  WiFi.softAP(ssid, password);
}

void loop(){

  if(UART.available())
  {
    UART.rxObj( rx_buffer );

    Serial.print( rx_buffer.x.gx);
    Serial.print(',');
    Serial.print( rx_buffer.x.gy);
    Serial.print(',');
    Serial.println( rx_buffer.x.gz);

    memset( tx_buffer, 0, tx_size );
    uint16_t checksum = 0;
    uint8_t value;
    uint8_t i = 0;
    
    // Get pointer to rx buffer, to write out each byte
    uint8_t * rx_buffer_ptr = (uint8_t*)&rx_buffer;

    // Start with Sync Word used by Telemetry Viewer
    tx_buffer[i++] = 0xAA;
 
    for(uint8_t j = 0; j < sizeof(tlm_data_t); j++ ){
      tx_buffer[i++] = *rx_buffer_ptr++;
      checksum += tx_buffer[i];
    }

    // Append checksum
    tx_buffer[i++] = checksum & 0xFF;
    tx_buffer[i++] = checksum >> 8;

    // Transmit UDP package
    UDP.beginPacket( UDP_address, UDP_port );
    UDP.write(tx_buffer, i );
    UDP.endPacket();
  }

  if( WiFi.softAPgetStationNum() > 0 )
    digitalWrite(led_pin, HIGH);  
  else
    digitalWrite(led_pin, LOW);  
}
