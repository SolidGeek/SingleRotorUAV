#include "src/sensors.h"
#include "src/control.h"
// #include "src/dshot.h"

Sensors sensors;
Control control;

void setup() {
    Serial.begin(115200);
    Serial.println("Welcome aboard the AAU Starliner.");
    Serial.println("Systems booting...");
    
    sensors.init();
    control.init();

    Serial.println("All systems nominal");
    // configESCs();
}

// Controller parameters (simple P-control)
float sp_roll = 0;
float sp_pitch = 0;

float err_roll = 0;
float err_pitch = 0;

float out_roll = 0;
float out_pitch = 0;

// Proportional gain
float Kp = 0.5;

void loop() {

    sensors.sample_imu();
    sensors.sample_lidar();
  
    err_roll = sp_roll - sensors.data.roll * (180/3.14);
    err_pitch = sp_pitch - sensors.data.pitch * (180/3.14);

    out_roll = Kp * err_roll;
    out_pitch = Kp * err_pitch;

    Serial.print(err_pitch);
    Serial.print(" ");
    Serial.println(err_roll);

    control.control_servo(0, out_pitch);
    control.control_servo(1, -out_roll);
    control.control_servo(2, -out_pitch);
    control.control_servo(3, out_roll);

    delay(5);

  /*  flow.readMotionCount(&deltaX, &deltaY);

  Serial.print(deltaX);
  Serial.print(",");
  Serial.println(deltaY);

  delay(5); */
  
  /* if (report_ID = IMU.getReadings())
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
      Serial.print(gx);
      Serial.print(",");
      Serial.println(gy);
    }
  }*/

}

/* 
void readRCReceiver( void ) {
  uint16_t roll = pulseIn( receiver_pins[0], HIGH);
  uint16_t pitch = pulseIn( receiver_pins[2], HIGH);
  uint16_t temp = pulseIn( receiver_pins[1], HIGH);
  uint16_t throttle = map(temp, 920, 1920, 0, 200) + 47;
}*/

/* 
void configESCs( void ) {

  Serial1.begin(115200);
  Serial2.begin(115200);

  // Configure the DSHOT outputs
  ESC.setup();

  // Config the ESCs
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_SPIN_DIRECTION_2 );
  ESC.setConfig( DSHOT_PORT_1, DSHOT_CMD_3D_MODE_OFF );

  ESC.setConfig( DSHOT_PORT_2, DSHOT_CMD_SPIN_DIRECTION_1 );
  ESC.setConfig( DSHOT_PORT_2, DSHOT_CMD_3D_MODE_OFF );

  delay(1000);

  // Arm all DSHOT ports / ESCs
  ESC.armMotors();
}*/
