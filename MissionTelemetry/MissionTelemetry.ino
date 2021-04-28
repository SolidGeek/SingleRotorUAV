// Load Wi-Fi library
#include <WiFi.h>
#include "SerialTransfer.h"

/* --- Data structs for telemetry --- */
typedef struct __attribute__ ((packed)){
    float gx, gy, gz;
    float roll, pitch, yaw;
    float ax, ay, az;
    float vx, vy;
    float z;
    struct{ // Bitfield, using 1 byte, to represent if new measurements are available
        uint8_t imu     : 1;
        uint8_t flow    : 1;
        uint8_t lidar   : 1;
    } status;
} sensor_data_t;

typedef struct __attribute__ ((packed)){
    float x, y, z;
    float vx, vy, vz;
} estimator_data_t;

typedef struct __attribute__ ((packed)){
  float a1, a2, a3, a4;   // Servo angles
  uint16_t dshot;         // Motor signal
} control_signal_t;

typedef struct __attribute__ ((packed)){
    sensor_data_t data;    // System output (states)
    estimator_data_t estimate;
    control_signal_t control; // System input  (actuation)
} tlm_data_t;

// Create struct to act as Serial RX buffer
tlm_data_t rx_buffer; 

const uint16_t tx_size = sizeof(tlm_data_t)+1;
const uint16_t udp_buffer_size = tx_size*1; // Room for 1 udp_packages
uint8_t tx_buffer[udp_buffer_size]; 
uint16_t tx_index = 0;

char udp_rx_buffer[255]; //buffer to hold incoming packet

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

  // To read UDP packages from PC (commands, external position data etc)
  UDP.begin(UDP_port);
}

void loop(){

  // Read UDP packages from client
  int udp_packet_size = UDP.parsePacket();

  if (udp_packet_size) {
    int len = UDP.read(udp_rx_buffer, 255);  
    // Send the data to Teensy
    Serial.write( udp_rx_buffer, len );
  }
  
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
