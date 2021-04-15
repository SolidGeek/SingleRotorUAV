// Load Wi-Fi library
#include <WiFi.h>

int led_pin = 12;

// Replace with your network credentials
const char* ssid     = "SR-UAV GUI";
const char* password = "123456789";

const char * UDP_address = "255.255.255.255";
const int UDP_port = 8080;

//The udp library class
WiFiUDP UDP;

void setup() {
  Serial.begin(921600);
  pinMode(led_pin, OUTPUT);

  WiFi.softAP(ssid, password);
}

char rx_buffer[400];

void loop(){

  if( Serial.available() ){
    memset( rx_buffer, 0, sizeof( rx_buffer ));
    Serial.readBytesUntil('\n', rx_buffer, sizeof( rx_buffer ));

    UDP.beginPacket( UDP_address, UDP_port );
    UDP.printf( rx_buffer );
    UDP.endPacket();
  }

  if( WiFi.softAPgetStationNum() > 0 )
    digitalWrite(led_pin, HIGH);  
  else
    digitalWrite(led_pin, LOW);  
}
