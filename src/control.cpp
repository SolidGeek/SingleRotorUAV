#include "control.h"

const uint16_t Control::servo_pins[] = { SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN };

Control::Control(){
    // Allocate four servo objects
    servos = new Servo[4];
}


void Control::init()
{
    for( uint8_t i = 0; i < 4; i++){
        // Configure min/max PWM for servos
        servos[i].attach( servo_pins[i], SERVO_MIN_TIMING, SERVO_MAX_TIMING);

        // Sensor all servos
        servos[i].writeMicroseconds(1550);
    }
}


void Control::control_servo( uint8_t index, float angle )
{
    // Scale angle, to get higher resolution on the map
    int32_t temp = (int32_t)(angle*1000);

    // Transform angle to PWM timing
    uint16_t timing = map( temp, -30*1000, 30*1000, SERVO_MIN_TIMING, SERVO_MAX_TIMING);

    // Write servo signal
    servos[index].writeMicroseconds(timing);
}

void Control::attitude( float roll, float pitch, float yaw, float gx, float gy, float gz ){

    // Load states into state-vector
    X << roll, pitch, yaw, gx, gy, gz;

    U = -K * X;
    
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.println(gy);


    control_servo(1, U(0) );
    control_servo(2, U(1) );
    control_servo(3, -U(2) );
    control_servo(0, -U(3) ); 

}