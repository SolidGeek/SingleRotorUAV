// Load Wi-Fi library
#include <WiFi.h>

int led_pin = 12;
bool led_state = LOW;

// Replace with your network credentials
const char* ssid     = "SR-UAV GUI";
const char* password = "123456789";

const char * udpAddress = "255.255.255.255";
const int udpPort = 8080;

//The udp library class
WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: "); Serial.println(IP);
}

void loop(){
  udp.beginPacket(udpAddress,udpPort);
  udp.printf("%d\n", random(100));
  udp.endPacket();
  delay(10);
  digitalWrite(led_pin, led_state);
  led_state = !led_state;
}
