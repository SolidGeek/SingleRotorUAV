#include "dshot.h"

DShot ESC1(1); // Output pin D2

uint8_t led_pin = 15;
bool led_state = LOW;

void setup() {

  pinMode(led_pin, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);
  
  Serial.println("DSHOT Test Teensy 4.0");

  ESC1.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  ESC1.write( 100, true );
  led_state = !led_state;
  digitalWrite(led_pin, led_state);
  delay(10);
  
}