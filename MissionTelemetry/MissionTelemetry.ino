// Load Wi-Fi library
#include <WiFi.h>
#include "SerialTransfer.h"

/* --- Data structs for telemetry --- */
typedef struct __attribute__ ((packed)) {
  float gx, gy, gz;
  float roll, pitch, yaw;
  float ax, ay, az;
  float vx, vy;
  float z;
  struct { // Bitfield, using 1 byte, to represent if new measurements are available
    uint8_t imu     : 1;
    uint8_t flow    : 1;
    uint8_t lidar   : 1;
  } status;
} sensor_data_t;

typedef struct __attribute__ ((packed)) {
  float x, y, z;
  float vx, vy, vz;
} estimator_data_t;

typedef struct __attribute__ ((packed)) {
  float a1, a2, a3, a4;   // Servo angles
  uint16_t dshot;         // Motor signal
} control_signal_t;

typedef struct __attribute__ ((packed)) {
  sensor_data_t data;    // System output (states)
  estimator_data_t estimate;
  control_signal_t control; // System input  (actuation)
} tlm_data_t;


typedef struct __attribute__ ((package)) {
  uint8_t command;
  float value;
} command_t;


// Create struct to act as Serial RX buffer
tlm_data_t telemetry_buffer;

command_t command_buffer;

// Create union
union{
  float data;
  uint8_t bytes[4];
} command_to_float; 

int16_t command_packet_size;

const uint16_t udp_tlm_size = sizeof(tlm_data_t) + 1;
uint8_t udp_tx_buffer[udp_tlm_size]; // buffer to hold bytes before transmission
uint8_t udp_rx_buffer[5];           // buffer to hold incoming packet

int led_pin = 12;

// Replace with your network credentials
const char* ssid     = "SR-UAV GUI";
const char* password = "123456789";

const char * UDP_address = "255.255.255.255";
const int UDP_port = 8888;

//The udp library class
WiFiUDP UDP;
SerialTransfer uart_transfer;

void setup() {
  pinMode(led_pin, OUTPUT);

  Serial.begin(921600);
  uart_transfer.begin(Serial);

  // To read and send UDP packages to and from PC (commands, external position data etc)
  WiFi.softAP(ssid, password);
  UDP.begin(UDP_port);
}

void loop() {

  // UDP Commands from PC 
  if ( command_packet_size = UDP.parsePacket() ){

    int8_t len = UDP.read( udp_rx_buffer, sizeof(udp_rx_buffer) );

    // If a correct sized data is read, send this to Teensy
    if( len == sizeof( udp_rx_buffer ) ){

      // First byte is command
      command_buffer.command = udp_rx_buffer[0];

      // Next 4 bytes is value. Convert to float
      memcpy( command_to_float.bytes, &udp_rx_buffer[1], 4 );
      command_buffer.value = command_to_float.data;

      // Send struct over UART to Teensy
      uart_transfer.sendDatum( command_buffer );
    }
  }

  // Serial structs from Teensy (telemetry)
  if ( uart_transfer.available() ) {
    // Read object into telemetry_buffer
    uart_transfer.rxObj( telemetry_buffer );

    // Transfer data from telemetry_buffer to udp_tx_buffer
    uint16_t tx_index = 0;
    uint8_t * buffer_ptr = (uint8_t*)&telemetry_buffer;
    
    // Start with Sync Word (used by Telemetry Viewer)
    udp_tx_buffer[tx_index++] = 0xAA;
    // Fill in the rest
    for (uint8_t i = 0; i < sizeof(tlm_data_t); i++ ) {
      udp_tx_buffer[tx_index++] = *buffer_ptr++;
    }

    // Send packet over UDP to PC
    UDP.beginPacket( UDP_address, UDP_port );
    UDP.write(udp_tx_buffer, udp_tlm_size );
    UDP.endPacket();
  }
  
  // Indicate that Wifi is connected to client
  if ( WiFi.softAPgetStationNum() > 0 ) {
    digitalWrite(led_pin, HIGH);
  } else
    digitalWrite(led_pin, LOW);
}
