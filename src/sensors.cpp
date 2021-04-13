#include "sensors.h"

Sensors::Sensors(void){
    imu = new BNO080(IMU_CS_PIN, IMU_WAK_PIN, IMU_INT_PIN, IMU_RST_PIN);
    flow = new PMW3901(FLOW_CS_PIN);
    lidar = new VL53L1X(); // I2C thus no CS
}

void Sensors::init(void){

    // Disable all SPI devices  before setup
    pinMode( IMU_CS_PIN, OUTPUT );
    pinMode( FLOW_CS_PIN, OUTPUT );
    digitalWrite( IMU_CS_PIN, HIGH );
    digitalWrite( FLOW_CS_PIN, HIGH );

    delay(100);

    // Initiate flow sensor
    if ( flow->begin() ) {
        flow->setLed( HIGH );
        status.flow = STATUS_READY;
    }else{
        status.flow = STATUS_FAILED_SETUP;
    }

    delay(100);

    // Init IMU
    if( imu->beginSPI() ){
        // Continous sample of sensors
        imu->enableGyro( 3 );  // 2.5ms / 400hz
        imu->enableRotationVector(5); // 5ms / 200Hz
        status.imu = STATUS_READY;
    }else{
        status.imu = STATUS_FAILED_SETUP;
    }

    delay(100);


    // Start I2C bus 1
    Wire1.begin();
    Wire1.setClock(400000); // use 400 kHz I2C

    // Set lidar to use I2C bus 1
    lidar->setBus(&Wire1);

    if ( lidar->init() ){
        lidar->setTimeout(0); // A value of 0 disables the timeout = Non-blocking
        lidar->setDistanceMode(VL53L1X::Medium); // Up to 2.9 meters
        lidar->setMeasurementTimingBudget(33000);
        lidar->startContinuous(33);

        status.lidar = STATUS_READY;
    }else{
        status.lidar = STATUS_FAILED_SETUP;
    }


}

void Sensors::sample_imu(){

    uint16_t report_ID;

    while ( report_ID = imu->getReadings() ) {
        // Read Attitude estimate
        if( report_ID == SENSOR_REPORTID_ROTATION_VECTOR ){
            data.roll   = imu->getRoll();   // Radians
            data.pitch  = imu->getPitch();  // Radians
            data.yaw    = imu->getYaw();    // Radians
        }

        // Read Raw Gyro data
        if( report_ID == SENSOR_REPORTID_GYROSCOPE ) {
            data.gx = LPF( imu->getGyroX(), data.gx, 0.4 ); // Radians / second
            data.gy = LPF( imu->getGyroY(), data.gy, 0.4 ); // Radians / second
            data.gz = LPF( imu->getGyroZ(), data.gz, 0.4 ); // Radians / second
        }
    }
}

void Sensors::sample_flow(){

}

void Sensors::sample_lidar(){

    if( lidar->dataReady() ){

        uint16_t value = lidar->read(false); // Non blocking

        data.z = ((float)value)/1000; // Convert to meters

    }

}

float Sensors::LPF( float new_sample, float old_sample, float alpha ){
    return ((alpha * new_sample) + (1.0-alpha) * old_sample);  
}