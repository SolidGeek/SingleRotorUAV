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


void Control::control_attitude( float roll, float pitch, float yaw, float gx, float gy, float gz ){

    // Load states into state-vector
    X << roll, pitch, yaw, gx, gy, gz;

    U = -K * X;
    
    /* Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.println(gy); */


    write_servo(1, -U(0) );
    write_servo(2, -U(1) );
    write_servo(3, U(2) );
    write_servo(0, U(3) ); 

}

void Control::get_rc_signals( void ) {

    // uint16_t temp_roll = pulseIn( receiver_pins[0], HIGH, 5000);
    // uint16_t temp_pitch = pulseIn( receiver_pins[2], HIGH, 5000);
    uint16_t temp_throttle = pulseIn( receiver_pins[1], HIGH, 10000);

    if( temp_throttle != 0){
        temp_throttle = constrain(temp_throttle, 920, 1920);
        SP_throttle = map(temp_throttle, 920, 1920, MOTOR_MIN_DSHOT, MOTOR_MAX_DSHOT); // Max at 1500, just to limit the possibility to overload motors.
    }
    

}

uint16_t Control::get_sp_throttle( void ){
    return SP_throttle;
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