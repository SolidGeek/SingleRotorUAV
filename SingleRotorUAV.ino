#include "src/dshot.h"

#define MOTOR_POLES 14    // Number of poles in connected motor
#define TLM_INTERVAL 500  // Interval in ms

/* Prepare two DSHOT outputs */
DShot ESC(2); 

uint16_t throttle = 200;
uint8_t tlm = 0;
uint64_t tlm_timer = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  Serial.println("DSHOT for Teensy 4.0");
  
  /* Configure the DSHOT outputs */
  ESC.setup();

  /* Retrieve previous settings over TLM wire */
  ESC.requestConfig( DSHOT_PORT_1, &Serial1 );
  ESC.requestConfig( DSHOT_PORT_2, &Serial2 );

  /* Config the ESCs */
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_SPIN_DIRECTION_2 ); 
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_3D_MODE_OFF );

  ESC.setConfig( DSHOT_PORT_2, DSHOT_CMD_SPIN_DIRECTION_1 );
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_3D_MODE_OFF );

  delay(1000);
  
  /* Arm all DSHOT ports / ESCs */
  ESC.armMotors();
  Serial.println("CAUTION: Motors armed");
}

void loop() {
  /* Request telemetry data each TLM_INTERVAL */
  if( millis() - tlm_timer > TLM_INTERVAL ){
    tlm = DSHOT_TLM_REQUEST;
  }else{
    tlm = DSHOT_TLM_NONE;
  }

  /* Write DSHOT signal */
  ESC.write(DSHOT_PORT_1, throttle, tlm);
  ESC.write(DSHOT_PORT_2, throttle, tlm);

  /* Read DSHOT telemetry from DSHOT_PORT_1 */
  if( ESC.readTelemetry( DSHOT_PORT_1, &Serial1 ) ){
    Serial.print("Voltage: ");
    Serial.print( ESC.getVoltage( DSHOT_PORT_1 ) );  
    Serial.print(" - ");
    Serial.print("Amps: ");
    Serial.print( ESC.getAmps( DSHOT_PORT_1 ) );  
    Serial.print(" - ");
    Serial.print("RPM: ");
    Serial.println( ESC.getRPM( DSHOT_PORT_1, MOTOR_POLES ) ); 
  }
  
  delay(10);

}
