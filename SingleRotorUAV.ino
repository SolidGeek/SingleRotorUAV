#include "src/config.h"
#include "src/sensors.h"
#include "src/control.h"
#include "src/logger.h"

Config conf;
Sensors sensors;
Control control;
Logger logger;

// Create struct for telemetry/logging
tlm_data_t tlm;

uint32_t control_timer = 0;
uint32_t flow_timer = 0;
uint16_t throttle = 0;

uint8_t rc_input1_pin = 19;
uint16_t rc_input1;
elapsedMicros rc_input1_timer;

void rc_input1_interrupt(){
   if( digitalReadFast(rc_input1_pin) == HIGH )
      rc_input1_timer = 0;
   else 
      rc_input1 = rc_input1_timer;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");

    attachInterrupt( rc_input1_pin, rc_input1_interrupt, CHANGE );

    // Prepare telemetry and logger
    logger.init();

    // Load configuration from memorys
    conf.load();

    // Configure servo offsets
    control.set_servo_offsets( conf.params.servo_offset);
    control.init();

    // Important to init this last, otherwise IMU's buffer overflow and goes into error state....
    sensors.init();

    Serial.println("Ready");
}

void loop() {

    // Sample IMU and LIDAR (sample rate is limited by sensors)
    sensors.sample_imu();
    sensors.sample_lidar();

    if( micros() - flow_timer >= 10000 ){
      flow_timer = micros();
      // The flow sensor outputs delta changes, needs to be sampled at semi-constant interval
      sensors.sample_flow(); 
    }

    if( micros() - control_timer >= 5000 ){
      control_timer = micros();

      // Save data in telemetry package before running estimator.
      tlm.data = sensors.data;
      
      // Run estimator and control
      sensors.run_estimator();
      control.control_hover( sensors.data.roll, sensors.data.pitch, 0, sensors.data.gx, sensors.data.gy, 0, sensors.estimate.z, 0 );

      // Save control and estimates to tlm.
      tlm.estimate = sensors.estimate;
      tlm.control = control.data;
      logger.write_esp( tlm );
      
      /* uint16_t temp = constrain(rc_input1, 930, 1910);
      throttle = map(temp, 930, 1910, MOTOR_MIN_DSHOT, MOTOR_MAX_DSHOT);
      control.write_motor( DSHOT_PORT_1, throttle );
      control.write_motor( DSHOT_PORT_2, throttle ); */
    }
}
