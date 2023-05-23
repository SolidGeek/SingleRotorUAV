#include "control.h"

DShot motors(2); 

const uint16_t Control::servo_pins[] = { SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN };

/* Interrupts to read RC control inputs */
const uint16_t rc_pins[] = {18, 19, 23, 1};
volatile uint16_t rc_input[4];
volatile elapsedMicros rc_input_timer[4];

void rc_input_isr0(){
    if( digitalReadFast(rc_pins[0]) == HIGH )
        rc_input_timer[0] = 0;
    else 
        rc_input[0] = rc_input_timer[0];
}

void rc_input_isr1(){
    if( digitalReadFast(rc_pins[1]) == HIGH )
        rc_input_timer[1] = 0;
    else 
        rc_input[1] = rc_input_timer[1];
}

void rc_input_isr2(){
    if( digitalReadFast(rc_pins[2]) == HIGH )
        rc_input_timer[2] = 0;
    else 
        rc_input[2] = rc_input_timer[2];
}

void rc_input_isr3(){
    if( digitalReadFast(rc_pins[3]) == HIGH )
        rc_input_timer[3] = 0;
    else 
        rc_input[3] = rc_input_timer[3];
}

Control::Control(){
    // Allocate four servo objects
    servos = new Servo[4];

    // Allocate DShot object, with two outputs; pin 8 and 9 (hardcoded)
    // motors = new DShot(2);
}


void Control::init( )
{
    
    attachInterrupt( rc_pins[0], rc_input_isr0, CHANGE );
    attachInterrupt( rc_pins[1], rc_input_isr1, CHANGE );
    attachInterrupt( rc_pins[2], rc_input_isr2, CHANGE );
    attachInterrupt( rc_pins[3], rc_input_isr3, CHANGE );

    // Configure servo motors
    for( uint8_t i = 0; i < 4; i++){
        // Configure min/max PWM for servos
        servos[i].attach( servo_pins[i], SERVO_MIN_TIMING, SERVO_MAX_TIMING);

        // Centre all servos
        write_servo( i , 0 ); // 0 degress
    }

    /* Configure the DSHOT outputs */
    motors.setup();

    // Configure BLDC motor drivers (ESC)
    motors.setConfig( DSHOT_PORT_1, DSHOT_CMD_SPIN_DIRECTION_2 ); // Depending on soldering of wires, this can be changed.
    motors.setConfig( DSHOT_PORT_1, DSHOT_CMD_3D_MODE_OFF );

    motors.setConfig( DSHOT_PORT_2, DSHOT_CMD_SPIN_DIRECTION_1 ); // Depending on soldering of wires, this can be changed.
    motors.setConfig( DSHOT_PORT_2, DSHOT_CMD_3D_MODE_OFF );

    delay(1000);
    /* Arm all DSHOT ports / ESCs */
    motors.armMotors();
}

void Control::set_servo_offsets( int16_t * offsets ){
    for (uint8_t i = 0; i < 4; i++)
    {   
        // Serial.println(offsets[i]);
        servo_offset[i] = offsets[i];
    }
}

void Control::write_motor( uint8_t index, uint16_t throttle ){

    if( throttle > 2000 )
        throttle = 2000;

    // Write motor throttle, and dont request telemetry = 0;
    motors.write( index, 47 + throttle, 1 );

}


void Control::write_servo( uint8_t index, float angle )
{
    // Scale angle, to get higher resolution on the map
    int32_t temp = (int32_t)(angle*1000);

    // Transform angle to PWM timing
    uint16_t timing = map( temp, -30*1000, 30*1000, SERVO_MIN_TIMING, SERVO_MAX_TIMING);

    // Write servo signal
    servos[index].writeMicroseconds(timing + servo_offset[index] );
}

void Control::write_servo_ms( uint8_t index, uint16_t ms ){
    servos[index].writeMicroseconds( ms );
}


void Control::read_control_input(){

    uint16_t input1 = constrain( rc_input[0], 1100, 1900);
    float z_ref = ((float)map(input1, 1100, 1900, 0, (uint16_t)(SETPOINT_MAX_Z*1000)))/1000;

    /* uint16_t input2 = constrain( rc_input[1], 1100, 1900);
    float roll_ref = ((float)map(input2, 1100, 1900, -(uint16_t)(SETPOINT_MAX_ROLL*1000), (uint16_t)(SETPOINT_MAX_ROLL*1000)))/1000;

    uint16_t input3 = constrain( rc_input[2], 1100, 1900);
    float pitch_ref = ((float)map(input3, 1100, 1900, -(uint16_t)(SETPOINT_MAX_PITCH*1000), (uint16_t)(SETPOINT_MAX_PITCH*1000)))/1000; */

    if( z_ref == 0 ){
        set_max_throttle(0); // Kill throttle
        reset_integral_action();

    }else{
        set_max_throttle(MOTOR_MAX_DSHOT);
    }

    // Only update setpoint if takeoff has been commanded
    // Do not interfere with the landing sequence
    if( status == CONTROL_STATUS_FLYING ){
        set_reference( SETPOINT_Z, z_ref );
    }
}


void Control::control_hover( float roll, float pitch, float yaw, float gx, float gy, float gz, float z, float vz ){
    
    Matrix<5,1> output; // Output vector
    Matrix<9,1> error; // State error vector
    Matrix<5,9> K = K_hover; 
    Matrix<9,1> ref = SP_hover;

    // Add the output of the position controller to the hover setpoint
    ref(0) += U_pos(0);
    ref(1) += U_pos(1);

    // Integral action for altitude (z)
    float error_z = ref(6) - z; 

    // If the throttle output is less than the allowed maximum, or the z error is lower than 0, compute the integral
    if( data.dshot < max_throttle || error_z < 0 ){

        // If the drone is landing, change how the integral behaves. 
        if( status == CONTROL_STATUS_LANDING ){
            if( z < 0.01 ){ // 10mm above ground
                // We have landing, stop engines
                status = CONTROL_STATUS_STATIONARY;
            }
            else if( z < 0.2 ){
                error_z = error_z * 1.2; // Reduce integral fast
            }else{
                error_z = error_z * 0.2;
            }
            
        }
        error_integral_z += error_z * CONTROL_LOOP_INTERVAL;
    }

    // Load states into state-vector (int_z = integral term)
    X << roll, pitch, yaw, gx, gy, gz, z, vz, 0;

    error = ref - X;
    error(8) = error_integral_z; // Insert integral term into the state-error vector

    // Special case for yaw:
    if( error(2) > PI )
        error(2) -= TWO_PI ;


    // If commanded to land, remove P-control from altitude and reduce integral-control
    if( status == CONTROL_STATUS_LANDING ){
        K(4,6) = 0;          // Remove P-gain
    }

    // Run Controller
    output = K * error; 

    // Filter servo outputs, to remove unwanted spikes
    U(0) = RateLimit( output(0), U(0), 0.3 );
    U(1) = RateLimit( output(1), U(1), 0.3 );
    U(2) = RateLimit( output(2), U(2), 0.3 );
    U(3) = RateLimit( output(3), U(3), 0.3 );

    // Saturate outputs to reduce control bleps
    U(0) = Limit( U(0), -20, 20 );
    U(1) = Limit( U(1), -20, 20 );
    U(2) = Limit( U(2), -20, 20 );
    U(3) = Limit( U(3), -20, 20 );

    U(4) = output(4);

}


void Control::set_reference( control_setpoint_t id, float value ){

    switch( id ){
        case SETPOINT_X: SP_pos(0) = value; break;
        case SETPOINT_Y: SP_pos(1) = value; break;
        case SETPOINT_Z: SP_hover(6) = value; break;
        case SETPOINT_ROLL: SP_hover(0) = value; break;
        case SETPOINT_PITCH: SP_hover(1) = value; break;
        case SETPOINT_YAW: SP_hover(2) = value; break;
    }

}

float Control::get_reference( control_setpoint_t id ){
    float value; 

    switch( id ){
        case SETPOINT_X: value = SP_pos(0); break;
        case SETPOINT_Y: value = SP_pos(1); break;
        case SETPOINT_Z: value = SP_hover(6); break;
        case SETPOINT_ROLL: value = SP_hover(0); break;
        case SETPOINT_PITCH: value = SP_hover(1); break;
        case SETPOINT_YAW: value = SP_hover(2); break;
    }

    return value;
}

void Control::control_position( float x, float y, float vx, float vy, float yaw ){

    Matrix<2,1> output;
    Matrix<6,1> error;

    // Load state vector
    X_pos << x, y, vx, vy, 0, 0;

    // Calculate state error
    error = SP_pos - X_pos;

    // Rotate error to body (assuming hover state, roll = 0, pitch = 0)
    error(0) = error(0)*cos(yaw) + error(1)*sin(yaw);  // x
    error(1) = error(1)*cos(yaw) - error(0)*sin(yaw);  // y
    error(2) = error(2)*cos(yaw) + error(3)*sin(yaw);  // vx
    error(3) = error(3)*cos(yaw) - error(2)*sin(yaw);  // vy

    // Integral computation and limit
    error_integral_x += error(0) * CONTROL_LOOP_INTERVAL;
    error_integral_y += error(1) * CONTROL_LOOP_INTERVAL;

    error_integral_x = Limit( error_integral_x, -pos_int_limit, pos_int_limit );
    error_integral_y = Limit( error_integral_y, -pos_int_limit, pos_int_limit );

    // Load integral error 
    error(4) = error_integral_x;
    error(5) = error_integral_y;

    // Run controller
    output = K_pos * error;

    // Limit position output to +-10 degress in roll and pitch 
    output(0) = Limit( output(0), -10 * DEG_TO_RAD, 10 * DEG_TO_RAD );
    output(1) = Limit( output(1), -10 * DEG_TO_RAD, 10 * DEG_TO_RAD );

    // Use the output of the positional controller as setpoints for the hover controller
    U_pos = output;
    
    // SP_hover(0) = output(0);
    // SP_hover(1) = output(1);
}

void Control::initiate_landing(){
    status = CONTROL_STATUS_LANDING;
    set_reference( SETPOINT_Z, 0 );
}

void Control::initiate_takeoff( float target_altitude ){
    status = CONTROL_STATUS_FLYING;
    set_reference( SETPOINT_Z, target_altitude );
}

void Control::run( sensor_data_t raw, estimator_data_t est ){

    uint16_t control_throttle;

    read_control_input();

    if( status == CONTROL_STATUS_STATIONARY ){
        control_hover( raw.roll, raw.pitch, 0, raw.gx, raw.gy, 0, 0, 0 );
        control_throttle = 0;
    }else{          
        control_position( est.x, est.y, est.vx, est.vy, raw.yaw );
        control_hover( raw.roll, raw.pitch, raw.yaw, raw.gx, raw.gy, raw.gz , est.z, est.vz  );
        control_throttle = (uint16_t)(MOTOR_KRPM_TO_DSHOT * U(4));
    }

    // Actuate servos
    write_servo(0, U(0) );
    write_servo(1, -U(1) );
    write_servo(2, -U(2) );
    write_servo(3, U(3) ); 

    data.a1 = U(0);
    data.a2 = U(1);
    data.a3 = U(2);
    data.a4 = U(3);

    // Limit output throttle
    if( control_throttle >  max_throttle )
        data.dshot = max_throttle;
    else
        data.dshot = control_throttle;

    if( get_reference( SETPOINT_Z ) == 0 ){

        if( input_zero_timer == false ){
            input_timer = millis();
            input_zero_timer = true;
        }
        
        if( millis() - input_zero_timer > 500 ){
            data.dshot = 0;
            input_zero_timer = false;
            input_zero_timer = millis();
        }
    }

    Serial.println(get_reference( data.dshot ));

    write_motor( DSHOT_PORT_1, data.dshot );
    write_motor( DSHOT_PORT_2, data.dshot );

}

void Control::reset_integral_action( void ){
    error_integral_x = 0;
    error_integral_y = 0;
    error_integral_z = 0;
}

void Control::set_max_throttle( uint16_t dshot ){
    max_throttle = dshot;
}

void Control::servo_calibration( int16_t * servo_offset ){
    Serial.println("Performing Servo Calibration. Write OK, when done. ");

    // First align servo motors with current offset.
    for (uint8_t i = 0; i < 4; i++)
    {
        write_servo_ms( i, SERVO_MID_TIMING + servo_offset[i] );
    }
    
    // Perform manual alignment
    for(uint8_t i = 0; i < 4; i++){

        Serial.print("Servo: "); Serial.println(i+1);

        uint8_t inputBuffer[5] = {'\0'};
        bool cal_done = false;

        while(!cal_done){

            uint8_t j = 0;

            while( Serial.available() ) {
                char c = Serial.read();
                inputBuffer[j++] = c;

                if( c == '\n' ){
                    if( strcmp( inputBuffer, "OK\n" ) == 0 ){
                        cal_done = true;
                    }else{
                        int16_t value = atoi( inputBuffer );
                        servo_offset[i] += value;

                        write_servo_ms( i, SERVO_MID_TIMING + servo_offset[i] );
                        Serial.println(servo_offset[i]);
                    }
                    j = 0;
                    memset(inputBuffer, 0, sizeof(inputBuffer));
                }
            }
        }
    }
}




float Control::RateLimit( float new_sample, float old_sample, float alpha ){
    return ((alpha * new_sample) + (1.0-alpha) * old_sample);  
}


float Control::Limit( float value, float min, float max ){
    float output;

    if( value > max ){
        output = max;
    }
    else if ( value < min ){
        output = min;
    }
    else{
        output = value;
    }

    return output;
}