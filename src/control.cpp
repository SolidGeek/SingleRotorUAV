#include "control.h"

DShot motors(2); 

const uint16_t Control::servo_pins[] = { SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN };
const uint16_t Control::receiver_pins[] = {18, 19, 23, 1};

Control::Control(){
    // Allocate four servo objects
    servos = new Servo[4];

    // Allocate DShot object, with two outputs; pin 8 and 9 (hardcoded)
    // motors = new DShot(2);
}


void Control::init( )
{
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
        Serial.println(offsets[i]);
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



void Control::control_hover( float roll, float pitch, float yaw, float gx, float gy, float gz, float z, float vz ){
    
    Matrix<5,1> output;

    // Integral action for altitude (z)
    float error_z = SP(6) - z; 
    if( data.dshot < max_throttle || error_z < 0 )
        error_integral_z += error_z * CONTROL_LOOP_INTERVAL;

    // Load states into state-vector (int_z = integral term)
    X << roll, pitch, yaw, gx, gy, gz, z, vz, 0;

    Xe = SP - X;
    Xe(8) = error_integral_z; // Insert integral term into the state-error vector

    // Special case for yaw:
    if( Xe(2) > PI )
        Xe(2) -= TWO_PI ;

    // Run Controller
    output = K * Xe; 

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


    Serial.print( output(0) );
    Serial.print( "," );
    Serial.println( U(0) );
    

    write_servo(1, -U(0) );
    write_servo(2, U(1) );
    write_servo(3, U(2) );
    write_servo(0, -U(3) ); 

    data.a1 = U(0);
    data.a2 = U(1);
    data.a3 = U(2);
    data.a4 = U(3);
    
    
    uint16_t control_throttle = (uint16_t)(MOTOR_KRPM_TO_DSHOT * U(4));

    if( control_throttle >  max_throttle )
        data.dshot = max_throttle;
    else
        data.dshot = control_throttle;

    write_motor( DSHOT_PORT_1, data.dshot );
    write_motor( DSHOT_PORT_2, data.dshot );

}

void Control::reset_integral_action( void ){

    error_integral_z = 0;

}

void Control::set_max_throttle( uint16_t dshot ){
    max_throttle = dshot;
}

void Control::control_attitude( float roll, float pitch, float yaw, float gx, float gy, float gz ){

    // Load states into state-vector
    X << roll, pitch, yaw, gx, gy, gz;

    U = -K * X;
    
    write_servo(1, -U(0) );
    write_servo(2, -U(1) );
    write_servo(3, U(2) );
    write_servo(0, U(3) ); 

    data.a1 = U(0);
    data.a2 = U(1);
    data.a3 = U(2);
    data.a4 = U(3);
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