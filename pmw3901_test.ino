#include "src/BNO080.h" 
#include "src/PMW3901.h"

#define FLOW_CS_PIN 6
#define IMU_CS_PIN 10
#define IMU_WAK_PIN 22
#define IMU_INT_PIN 21
#define IMU_RST_PIN 20

BNO080 IMU;
PMW3901 FLOW(FLOW_CS_PIN);

uint16_t report_ID = 0;
float gx, gy, gz = 0;
float roll, pitch, yaw = 0;
uint8_t accuracy; 

void setup() {

  Serial.begin(115200);

  pinMode( IMU_CS_PIN, OUTPUT );
  pinMode( FLOW_CS_PIN, OUTPUT );
  digitalWrite( IMU_CS_PIN, HIGH );
  digitalWrite( FLOW_CS_PIN, HIGH );

  delay(100);

  /* IMU.enableDebugging(Serial);
  if(IMU.beginSPI(IMU_CS_PIN, IMU_WAK_PIN, IMU_INT_PIN, IMU_RST_PIN) == false){
    Serial.println(F("BNO080 over SPI not detected."));
  } */
  
  if (!FLOW.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    //while(1);
  }
  
  

  /* delay(1000);

  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1);
  }*/

  // IMU.enableGyro( 3 );  // 2.5ms / 400hz
  // IMU.enableRotationVector(5); // 5ms / 200Hz
  
}

void loop() {
  /* if (report_ID = IMU.getReadings())
  {
    Serial.println(report_ID);
    // Attitude estimate
    if( report_ID == SENSOR_REPORTID_ROTATION_VECTOR ){
      roll = IMU.getRoll();
      pitch = IMU.getPitch();
      yaw = IMU.getYaw();
    }

    // Raw Gyro data
    if( report_ID == SENSOR_REPORTID_GYROSCOPE ) {
      IMU.getGyro(gx, gy, gz, accuracy);
      Serial.print(gx);
      Serial.print(",");
      Serial.println(gy);
    }
  }*/

  digitalWrite( FLOW_CS_PIN, HIGH );
  delay(1000);
  digitalWrite( FLOW_CS_PIN, LOW );
  delay(1000);
}
