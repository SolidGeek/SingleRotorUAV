#include "src/dshot.h"

#define MOTOR_POLES 14    // Number of poles in connected motor
#define TLM_INTERVAL 500  // Interval in ms

/* Prepare one DSHOT outputs */
DShot ESC(2); 

uint16_t throttle = 47;
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

  /* Config the ESCs */
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_SPIN_DIRECTION_1 ); 
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_3D_MODE_OFF );

  ESC.setConfig( DSHOT_PORT_2, DSHOT_CMD_SPIN_DIRECTION_1 ); 
  ESC.setConfig( DSHOT_PORT_2, DSHOT_CMD_3D_MODE_OFF );

  delay(1000);
  
  /* Arm all DSHOT ports / ESCs */
  ESC.armMotors();
  Serial.println("DSHOT \t VOLT1 \t AMP1 \t RMP1 \t TEMP1 \t VOLT2 \t AMP2 \t RPM2 \t TEMP2");
}

uint8_t inputBuffer[5] = {'\0'};
uint8_t i;

bool newMeasurement = false;
uint64_t tlmPrintTimer = 0;

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
  ESC.readTelemetry( DSHOT_PORT_1, &Serial1) ;
  ESC.readTelemetry( DSHOT_PORT_2, &Serial2) ;


  if( millis() - tlmPrintTimer > 2000 && newMeasurement == true){
      Serial.print((String)ESC.getVoltage( DSHOT_PORT_1 ) + "\t");    
      Serial.print((String)ESC.getAmps( DSHOT_PORT_1 ) + "\t");
      Serial.print((String)ESC.getRPM( DSHOT_PORT_1, MOTOR_POLES) + "\t");
      Serial.print((String)ESC.getTemp( DSHOT_PORT_1 ) + "\t");
      Serial.print((String)ESC.getVoltage( DSHOT_PORT_2 ) + "\t");
      Serial.print((String)ESC.getAmps( DSHOT_PORT_2 ) + "\t");
      Serial.print((String)ESC.getRPM( DSHOT_PORT_2, MOTOR_POLES) + "\t");
      Serial.print((String)ESC.getTemp( DSHOT_PORT_2 ) + "\t");
      Serial.println();

      newMeasurement = false;
  }

  
  while( Serial.available() ) {
    char c = Serial.read();
    inputBuffer[i++] = c;

    if( c == '\n' ){
      if( strcmp( inputBuffer, "OK\n" ) == 0 ){
        throttle = throttle + 50;
      }else{
        long input = atoi( inputBuffer );
        throttle = input + 47;
        Serial.println();
      }

      Serial.print((String)(throttle-47) + "\t");
      tlmPrintTimer = millis();
      newMeasurement = true;
      // Serial.println( "Throttle " + (String)(throttle-47) );

      i = 0;
      break;
    } 
  }
  
  delay(10);

}
