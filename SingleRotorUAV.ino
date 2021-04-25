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

uint8_t rc_input1_pin = 19;
uint16_t rc_input1;
elapsedMicros rc_input1_timer;

uint16_t throttle = 0;

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

    // control.servo_calibration( conf.params.servo_offset );
    // conf.save();

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
      control.control_position( sensors.estimate.x, sensors.estimate.y, sensors.estimate.vx, sensors.estimate.vy );
      control.control_hover( sensors.data.roll, sensors.data.pitch, sensors.data.yaw, sensors.data.gx, sensors.data.gy, sensors.data.gz , sensors.estimate.z, sensors.estimate.vz );

      // Manuel throttle override
      uint16_t temp = constrain(rc_input1, 930, 1910);
      uint16_t rc_throttle = map(temp, 930, 1910, MOTOR_MIN_DSHOT, MOTOR_MAX_DSHOT);
      control.set_max_throttle(rc_throttle);

      if( rc_throttle == 0 ){
        control.reset_integral_action();
      }

      // Save control and estimates to tlm.
      tlm.estimate = sensors.estimate;
      tlm.control = control.data;
      logger.write_esp( tlm );

    }
}
