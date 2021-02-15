#include "dshot.h"

#define DSHOT_N 2

DShot ESC(DSHOT_N); // Prepare two DSHOT outputs

uint16_t throttle[DSHOT_N]  = {128, 254};
uint16_t arming[DSHOT_N]    = {0, 0};
uint8_t tlm[DSHOT_N]        = {0, 0};

void setup() {
  Serial.begin(115200);
  Serial.println("--- DSHOT Teensy 4.0 ---");

  // Configure the DSHOT outputs
  ESC.setup();
  Serial.println("STATUS: DSHOT outputs configured");
  // Arm motors
  // Send command DSHOT_CMD_ARMING 500 times
  for ( int i = 0; i < 500; i++ )  {
    ESC.write( arming, tlm );
    delayMicroseconds( 2000 );
  } 

  Serial.println("CAUTION: Motors armed");

  delay(100);
}

void loop() {

  ESC.write( throttle, tlm );
  delayMicroseconds( 1000 );
  
}
