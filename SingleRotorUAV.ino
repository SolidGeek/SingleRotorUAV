#include "src/dshot.h"

#define DSHOT_N 2

DShot ESC(DSHOT_N); // Prepare two DSHOT outputs

uint16_t throttle = 60;
uint8_t tlm = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("--- DSHOT Teensy 4.0 ---");

  Serial1.begin(115200);

  // Configure the DSHOT outputs
  ESC.setup();

  ESC.arm_motor( 0 );
  
  delay(1000);

  ESC.set_rotation_reverse( 0 );
  
  // Serial.println("STATUS: DSHOT outputs configured");
  // delay(500);

  // Serial.println("CAUTION: Motors armed");
  delay(1000);
  
  ESC.request_esc_info( 0, &Serial1 );
  
}

void loop() {


}
