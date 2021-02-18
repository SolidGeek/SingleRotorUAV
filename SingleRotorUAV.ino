#include "src/dshot.h"

#define MOTOR_POLES 14    // Number of poles in connected motor
#define TLM_INTERVAL 500  // Interval in ms

/* Prepare one DSHOT outputs */
DShot ESC(1); 

uint16_t throttle = 47;
uint8_t tlm = 0;
uint64_t tlm_timer = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println("DSHOT for Teensy 4.0");
  
  /* Configure the DSHOT outputs */
  ESC.setup();

  /* Retrieve previous settings over TLM wire */
  ESC.requestConfig( DSHOT_PORT_1, &Serial1 );

  /* Config the ESCs */
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_SPIN_DIRECTION_2 ); 
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_3D_MODE_OFF );

  delay(1000);
  
  /* Arm all DSHOT ports / ESCs */
  ESC.armMotor( DSHOT_PORT_1 );
  Serial.println("CAUTION: Motors armed");
}

uint8_t inputBuffer[5] = {'\0'};
uint8_t i;

void loop() {
  /* Request telemetry data each TLM_INTERVAL */
  if( millis() - tlm_timer > TLM_INTERVAL ){
    tlm = DSHOT_TLM_REQUEST;
  }else{
    tlm = DSHOT_TLM_NONE;
  }

  /* Write DSHOT signal */
  ESC.write(DSHOT_PORT_1, throttle, tlm);

  /* Read DSHOT telemetry from DSHOT_PORT_1 */
  if( ESC.readTelemetry( DSHOT_PORT_1, &Serial1) ) {

    Serial.print("VOLT: " + (String)ESC.getVoltage( DSHOT_PORT_1 ) + "\t");
    Serial.print("AMP: " + (String)ESC.getAmps( DSHOT_PORT_1 ) + "\t");
    Serial.print("RPM: " + (String)ESC.getRPM( DSHOT_PORT_1, MOTOR_POLES) + "\t");
    Serial.print("TEMP: " + (String)ESC.getTemp( DSHOT_PORT_1 ) + "\t");
    Serial.println(); 
    
  }
  
  while( Serial.available() ) {
    char c = Serial.read();
    inputBuffer[i++] = c;

    if( c == '\n' ){
      long input = atoi( inputBuffer );

      throttle = input + 47;
      Serial.println( "Throttle output: " + (String)input );
      i = 0;
      break;
    } 
  }
  
  delay(10);

}
