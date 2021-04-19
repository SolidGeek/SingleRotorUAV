#include "src/config.h"
#include "src/sensors.h"
#include "src/control.h"
#include "src/logger.h"

Config conf;
Sensors sensors;
Control control;
Logger logger;

uint32_t control_timer = 0;
uint32_t tlm_timer = 0;

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
    Serial.println("Welcome aboard the AAU Starliner.");
    Serial.println("Systems booting...");

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

    
    Serial.println("Systems ready");
}

bool logging_initiated = false;

void loop() {

    // Sample as often as possible!
    sensors.sample_imu();
    sensors.sample_lidar();

    if( micros() - control_timer >= 5000 ){
      control_timer = micros();

      control.control_attitude( sensors.data.roll, sensors.data.pitch, 0, sensors.data.gx, sensors.data.gy, 0 );
      
      uint16_t temp = constrain(rc_input1, 930, 1910);
      throttle = map(temp, 930, 1910, MOTOR_MIN_DSHOT, MOTOR_MAX_DSHOT);

      if( throttle > 100 && !logging_initiated ){
        logging_initiated = true;
        logger.start_log();
        Serial.println("Started Logging");
      }
      
      if ( throttle < 50 && logging_initiated ){
        logger.end_log();
        Serial.println("Logging Ended");
      }

      /* control.write_motor( DSHOT_PORT_1, throttle );
      control.write_motor( DSHOT_PORT_2, throttle ); */

      // The flow sensor outputs delta changes, needs to be sampled consistently
      sensors.sample_flow();
      
    }

    // Send telemetry at 40 Hz
    if( micros() - tlm_timer > 25000 ) {
      // Send telemetry by UART to ESP32
      tlm_timer = micros();
      logger.write_esp( sensors.data, control.data );
      // logger.write_sd( sensors.data, control.data );
    }

}

/* 
void readRCReceiver( void ) {
  uint16_t roll = pulseIn( receiver_pins[0], HIGH);
  uint16_t pitch = pulseIn( receiver_pins[2], HIGH);
  uint16_t temp = pulseIn( receiver_pins[1], HIGH);
  uint16_t throttle = map(temp, 920, 1920, 0, 200) + 47;
}*/
