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

const uint16_t tx_size = sizeof(tlm_data_t)+1;
const uint16_t udp_buffer_size = tx_size*1; // Room for 1 udp_packages
uint8_t tx_buffer[udp_buffer_size]; 
uint16_t tx_index = 0;


int led_pin = 12;

// Replace with your network credentials
const char* ssid     = "SR-UAV GUI";
const char* password = "123456789";

const char * UDP_address = "255.255.255.255";
const int UDP_port = 8888;

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

  if( tx_index >= udp_buffer_size ){

    // Send that big boi
    UDP.beginPacket( UDP_address, UDP_port );
    UDP.write(tx_buffer, udp_buffer_size );
    UDP.endPacket();
    
    memset( tx_buffer, 0, tx_size );
    tx_index = 0;
  }

  if( UART.available() ){
    // Read object into rx_buffer
    UART.rxObj( rx_buffer );

    // Transfer data from rx_buffer to tx_buffer 
    uint8_t * rx_buffer_ptr = (uint8_t*)&rx_buffer;

    // Start with Sync Word used by Telemetry Viewer
    tx_buffer[tx_index++] = 0xAA;
    // Fill in the rest
    for(uint8_t i = 0; i < sizeof(tlm_data_t); i++ ){
      tx_buffer[tx_index++] = *rx_buffer_ptr++;
    }
  }

  if( WiFi.softAPgetStationNum() > 0 ){
    digitalWrite(led_pin, HIGH);  
  }else
    digitalWrite(led_pin, LOW);  
}
