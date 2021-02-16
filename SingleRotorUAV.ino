#include "src/dshot.h"

#define DSHOT_N 2

DShot ESC(DSHOT_N); // Prepare two DSHOT outputs

uint16_t throttle = 60;
uint8_t tlm = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("--- DSHOT Teensy 4.0 ---");

  Serial1.begin(115200);

  // Configure the DSHOT outputs
  ESC.setup();

  // Get last settings. Cannot be called after settings are changed (dunno why)
  ESC.requestConfig( 0, &Serial1 );

  ESC.setConfig( 0, DSHOT_CMD_SPIN_DIRECTION_1 );
  ESC.setConfig( 0, DSHOT_CMD_3D_MODE_OFF );

  delay(1000);

  Serial.println("CAUTION: Arming motors");
  ESC.armMotor( 0 );

}

void loop() {
  
  ESC.write(0, throttle, tlm);
  delay(10);

}
