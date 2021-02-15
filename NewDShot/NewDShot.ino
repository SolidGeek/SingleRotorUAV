#include "dshot.h"

DShot ESC1(1); // Output pin D2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);
  
  Serial.println("DSHOT Test Teensy 4.0");

  ESC1.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  ESC1.write( 100, true );
  Serial.println("Write");
  delay(100);
}
