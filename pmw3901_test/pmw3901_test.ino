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
  Serial.println("Ready");

  delay(1000);

  pinMode( IMU_CS_PIN, OUTPUT );
  pinMode( FLOW_CS_PIN, OUTPUT );
  digitalWrite( IMU_CS_PIN, HIGH );
  digitalWrite( FLOW_CS_PIN, HIGH );

  delay(100);

  // IMU.enableDebugging(Serial);
  if(IMU.beginSPI(IMU_CS_PIN, IMU_WAK_PIN, IMU_INT_PIN, IMU_RST_PIN) == false){
    Serial.println("BNO080 over SPI not detected.");
  }
  
  if (!FLOW.begin()) {
    Serial.println("Initialization of the flow sensor failed");
  }
  

  IMU.enableGyro( 3 );  // 2.5ms / 400hz
  IMU.enableRotationVector(5); // 5ms / 200Hz

  Serial.println("Setup done");
  
}

int16_t deltaX; int16_t deltaY;

void loop() {

  Serial.println("LOOP");

  FLOW.readMotionCount(&deltaX, &deltaY);

  Serial.println("FLOW");
  Serial.print(deltaX);
  Serial.print(",");
  Serial.println(deltaY);

  if (report_ID = IMU.getReadings())
  {
    // Attitude estimate
    if( report_ID == SENSOR_REPORTID_ROTATION_VECTOR ){
      roll = IMU.getRoll();
      pitch = IMU.getPitch();
      yaw = IMU.getYaw();
    }

    // Raw Gyro data
    if( report_ID == SENSOR_REPORTID_GYROSCOPE ) {
      IMU.getGyro(gx, gy, gz, accuracy);
      Serial.println("GYRO");
      Serial.print(gx);
      Serial.print(",");
      Serial.println(gy);
    }
  }

}
