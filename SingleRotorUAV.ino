#include "src/dshot.h"

// Prepare two DSHOT outputs
#define DSHOT_OUTPUTS 2
DShot ESC(DSHOT_OUTPUTS); 

uint16_t throttle = 200;
uint8_t tlm = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("--- DSHOT Teensy 4.0 ---");

  Serial1.begin(115200);
  Serial2.begin(115200);
  
  // Configure the DSHOT outputs
  ESC.setup();

  // Get last settings. Cannot be called after settings are changed (dunno why)
  // ESC.requestConfig( 0, &Serial1 );
  // ESC.requestConfig( 1, &Serial2 );

  // Config DSHOT ESCs
  ESC.setConfig( 0, DSHOT_CMD_SPIN_DIRECTION_2 ); 
  ESC.setConfig( 0, DSHOT_CMD_3D_MODE_OFF );

  ESC.setConfig( 1, DSHOT_CMD_SPIN_DIRECTION_1 );
  ESC.setConfig( 1, DSHOT_CMD_3D_MODE_OFF );

  delay(1000);

  Serial.println("CAUTION: Arming motors");
  ESC.armMotors( );
}

void loop() {
  
  ESC.write(0, throttle, tlm);
  ESC.write(1, throttle, tlm);
  delay(10);

}
