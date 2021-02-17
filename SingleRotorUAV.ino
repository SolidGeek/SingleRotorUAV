#include "src/dshot.h"

// Prepare two DSHOT outputs
#define DSHOT_OUTPUTS 2
#define DSHOT_MOTOR_POLES 14
#define DSHOT_TLM_INTERVAL 500 // Milisecond (ms)

DShot ESC(DSHOT_OUTPUTS); 

uint16_t throttle = 450;
uint8_t tlm = 0;
uint64_t tlm_timer = 0;

DSHOT_telemetry * data; 

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

  if( millis() - tlm_timer > DSHOT_TLM_INTERVAL ){
    tlm = DSHOT_TLM_REQUEST;
  }else{
    tlm = DSHOT_TLM_NONE;
  }
  
  ESC.write(0, throttle, tlm);
  ESC.write(1, throttle, tlm);

  if( ESC.readTelemetry( 0, &Serial1 ) ){
    Serial.print("Voltage: ");
    Serial.print(ESC.tlm_data[0].voltage );  
    Serial.print(" - ");
    Serial.print("Amps: ");
    Serial.println(ESC.tlm_data[0].ampHours ); 
  }
  
  delay(10);

}
