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

uint16_t throttle = 0;
int cmd;


bool enable_loiter = false;
float loiter_radius = 0.5; // Meter
float loiter_time = 20; // Seconds
uint32_t loiter_timer = 0;


const int path_count = 8;
float xref[path_count] = {0, 1, 1, -1, -1, 1, 1, 0};
float yref[path_count] = {0, 0, -1, -1, 1, 1, 0, 0};
bool follow_path = false;



void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");

    // attachInterrupt( rc_input1_pin, rc_input1_interrupt, CHANGE );

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

uint32_t last_vicon = 0;

void loop() {

    // Sample IMU and LIDAR (sample rate is limited by sensors)
    sensors.sample_imu();
    sensors.sample_lidar();

    cmd = comm.read_udp_commands();
    if( cmd >= 0 ){
      float * values = comm.get_command_values();
      switch( cmd ){

        case COMMAND_LAND:
          control.initiate_landing();
          enable_loiter = false;
        break;

        case COMMAND_TAKEOFF:
          control.initiate_takeoff( values[0] );
          enable_loiter = false;
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
          enable_loiter = false;
        break;  

        case DATA_UPDATE_POS:
          sensors.update_pos( values[0], values[1] );

          last_vicon = millis();
        break;

        case COMMAND_LOITER: 
          // Start following circle path until told to stop / land.
          follow_path = true;
          
        default:
          /* Serial.print("Unknown command: ");
          Serial.print( cmd ); Serial.print( " - With data: " );
          Serial.println( value ); */
        break;
      }
    }

    if( micros() - flow_timer >= 20000 ){
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
      
      if( follow_path ){
        path_follower();
      }

      control.run( sensors.data, sensors.estimate );

      // Save control and estimates to tlm. Write telemetry
      tlm.estimate = sensors.estimate;
      tlm.control = control.data;
      comm.write_udp_telemetry( tlm );

    }
}

void generate_loiter(){

  static float xpre = 0;
  static float ypre = 0;
  
  float xref, yref, dx, dy, yawref;

  float t = (float)(millis() - loiter_timer)/1000; // Seconds since start
  float k = TWO_PI / loiter_time;

  // Generate circular path for position controller
  xref = sin( k * t ) * loiter_radius;
  yref = cos( k * t ) * loiter_radius - loiter_radius; // Minus sin (clockwise flight)

  dx = xref - xpre;
  dy = yref - ypre;
  
  yawref = atan2( dy, dx );

  control.set_reference( SETPOINT_X, xref ); 
  control.set_reference( SETPOINT_Y, yref ); 
  control.set_reference( SETPOINT_YAW, 0 ); 

  xpre = xref;
  ypre = yref;

  // Serial.println( (String)xref + "," + (String)yref + "," + (String)yawref );

}


void path_follower(){

  const float deadzone = 0.05;
  const int stand_time = 2000; // 2 seconds
  const float k = 0.5;

  static int coordinate_id = 0;
  static uint32_t timer = 0;
  static bool reached = false;

  float xr = k*xref[coordinate_id];
  float yr = k*yref[coordinate_id];

  if( ((xr - sensors.estimate.x) < deadzone && (yr - sensors.estimate.y) < deadzone ) && !reached ){
    reached = true;
    timer = millis();
  }else{
    control.set_reference(SETPOINT_X, xr);
    control.set_reference(SETPOINT_Y, yr);  
  }

  if( reached ) {
    if( millis() - timer > stand_time ){
      Serial.println("Reached");
      // NEXT POSITION
      coordinate_id++;
      reached = false;  
      timer = 0;
      
      if( coordinate_id >= path_count ){
        Serial.println("Reset");
        coordinate_id = 0;
        follow_path = false;
      }
    }
  }

    
}
