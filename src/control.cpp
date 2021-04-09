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


void Control::init()
{
    // Configure servo motors
    for( uint8_t i = 0; i < 4; i++){
        // Configure min/max PWM for servos
        servos[i].attach( servo_pins[i], SERVO_MIN_TIMING, SERVO_MAX_TIMING);

        // Sensor all servos
        servos[i].writeMicroseconds(1550);
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

void Control::set_motor( uint8_t index, uint16_t throttle ){

    if( throttle > 2000 )
        throttle = 2000;

    // Write motor throttle, and dont request telemetry = 0;
    motors.write( index, 47 + throttle, 1 );

}

void Control::set_servo( uint8_t index, float angle )
{
    // Scale angle, to get higher resolution on the map
    int32_t temp = (int32_t)(angle*1000);

    // Transform angle to PWM timing
    uint16_t timing = map( temp, -30*1000, 30*1000, SERVO_MIN_TIMING, SERVO_MAX_TIMING);

    // Write servo signal
    servos[index].writeMicroseconds(timing);
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


    set_servo(1, U(0) );
    set_servo(2, U(1) );
    set_servo(3, -U(2) );
    set_servo(0, -U(3) ); 

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