#include "src/config.h"
#include "src/sensors.h"
#include "src/control.h"
#include "src/communication.h"

Config conf;
Sensors sensors;
Control control;
Communication comm;

// Create struct for telemetry/logging
tlm_data_t tlm;

uint32_t control_timer = 0;
uint32_t flow_timer = 0;

uint8_t rc_input1_pin = 19;
uint16_t rc_input1;
elapsedMicros rc_input1_timer;

uint16_t throttle = 0;
int cmd;

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

    // Prepare communication for telemetry and commands
    comm.init();

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

    cmd = comm.read_udp_commands();
    if( cmd >= 0 ){
      float value = comm.get_command_value();
      switch( cmd ){

        case COMMAND_LAND:
          control.initiate_landing();
        break;

        case COMMAND_TAKEOFF:
          control.initiate_takeoff( value );
        break;
        
        case COMMAND_SETPOINT_X:
          control.set_position_x( value );
        break;

        case COMMAND_SETPOINT_Y:
          control.set_position_y( value );
        break;
        
        case COMMAND_SET_ORIGIN:
          sensors.set_origin();
          control.reset_integral_action();
        break;  

        case DATA_POSITION_X:
          sensors.update_pos_x( value );
        break;

        case DATA_POSITION_Y:
          sensors.update_pos_y( value );
        break;
        
        default:
          /* Serial.print("Unknown command: ");
          Serial.print( cmd ); Serial.print( " - With data: " );
          Serial.println( value ); */
        break;
      }
    }

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

      control.run( sensors.data, sensors.estimate );
      
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
      comm.write_udp_telemetry( tlm );

    }
}
