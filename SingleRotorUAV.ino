#include "src/config.h"
#include "src/sensors.h"
#include "src/control.h"
#include "src/communication.h"

Config conf;
Sensors sensors;
Control control;
Communication comm;

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

    comm.init();

    attachInterrupt( rc_input1_pin, rc_input1_interrupt, CHANGE );

    // Load configuration from memorys
    conf.load();

    control.set_servo_offsets( conf.params.servo_offset);
    control.init();
    //control.servo_calibration( conf.params.servo_offset );
    //conf.save();

    // Important to init this last, otherwise IMU's buffer overflow and goes into error state....
    sensors.init();
    Serial.println("System ready");
}

void loop() {

    // Sample as often as possible!
    sensors.sample_imu();

    if( micros() - control_timer >= 5000 ){
      control_timer = micros();

      // Only sample at 200Hz
      // sensors.sample_lidar();
      
      control.control_attitude( sensors.data.roll, sensors.data.pitch, 0, sensors.data.gx, sensors.data.gy, 0 );
      
      uint16_t temp = constrain(rc_input1, 930, 1910);
      throttle = map(temp, 930, 1910, MOTOR_MIN_DSHOT, MOTOR_MAX_DSHOT);

      control.write_motor( DSHOT_PORT_1, throttle );
      control.write_motor( DSHOT_PORT_2, throttle );

      // Send telemetry by UART to ESP32
      comm.send_tlm( sensors.data, control.data);
    }


  /*  flow.readMotionCount(&deltaX, &deltaY); */

}

/* 
void readRCReceiver( void ) {
  uint16_t roll = pulseIn( receiver_pins[0], HIGH);
  uint16_t pitch = pulseIn( receiver_pins[2], HIGH);
  uint16_t temp = pulseIn( receiver_pins[1], HIGH);
  uint16_t throttle = map(temp, 920, 1920, 0, 200) + 47;
}*/
