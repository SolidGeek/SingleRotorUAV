#include "src/config.h"
#include "src/sensors.h"
#include "src/control.h"
#include "src/communication.h"

#define PERFORM_SERVO_CALIBRATION false

Config conf;
Sensors sensors;
Control control;
Communication comm;

// Create struct for telemetry/logging
tlm_data_t tlm;

uint32_t control_timer = 0;
uint32_t sensor_timer = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");

    // Prepare communication for telemetry and commands
    comm.init();

    // Load configuration from memorys
    conf.load();

    // Configure servo offsets
    control.set_servo_offsets( conf.params.servo_offset );
    control.init();

    if( PERFORM_SERVO_CALIBRATION ){
      control.servo_calibration( conf.params.servo_offset );
      conf.save();
    }

    // Important to init this last, otherwise IMU's buffer overflow and goes into error state....
    sensors.init();
    
    Serial.println("Ready");
}

void loop() {

    // Sample IMU (sample rate is limited by sensors)
    sensors.sample_imu();

    if( micros() - sensor_timer >= 20000 ){
      sensor_timer = micros();

      sensors.sample_lidar();
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

      // Save control and estimates to tlm. Write telemetry
      tlm.estimate = sensors.estimate;
      tlm.control = control.data;
      comm.write_udp_telemetry( tlm );

    }

    // Listen for commands from ground station
    uint8_t cmd = comm.read_udp_commands();
    
    if( cmd >= 0 ){
      float * values = comm.get_command_values();
      
      switch( cmd ){
        case COMMAND_LAND:
          control.initiate_landing();
        break;

        case COMMAND_TAKEOFF:
          control.initiate_takeoff( 0 );
        break;
        
        case COMMAND_SET_X:
          control.set_reference( SETPOINT_X, values[0] );
        break;

        case COMMAND_SET_Y:
          control.set_reference( SETPOINT_Y, values[0] );
        break;

        case COMMAND_SET_ROLL:
          control.set_reference( SETPOINT_ROLL, values[0] );
        break;

        case COMMAND_SET_PITCH:
          control.set_reference( SETPOINT_PITCH, values[0] );
        break;

        case COMMAND_SET_YAW:
          control.set_reference( SETPOINT_YAW, values[0] );
        break;
        
        case COMMAND_SET_ORIGIN:
          sensors.set_origin();
          control.reset_integral_action();
        break;  

        case DATA_UPDATE_POS:
          sensors.update_pos( values[0], values[1] );
        break;
          
        default:
          /* Serial.print("Unknown command: ");
          Serial.print( cmd ); Serial.print( " - With data: " );
          Serial.println( value ); */
        break;
      }
    }
}
