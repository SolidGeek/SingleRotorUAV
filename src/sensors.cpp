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
        imu->enableGyro(5);  // 5ms / 200hz
        imu->enableLinearAccelerometer(5);  // 5ms / 200hz
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

sensor_data_t Sensors::get_samples( void ){

    sensor_data_t temp = data;
    data.status = {};
    
    return temp;

}

void Sensors::sample_imu(){

    uint16_t report_ID;

    while ( report_ID = imu->getReadings() ) {
        // Read Attitude estimate
        if( report_ID == SENSOR_REPORTID_ROTATION_VECTOR ){
            data.roll   = imu->getRoll();   // Radians
            data.pitch  = imu->getPitch();  // Radians
            data.yaw    = imu->getYaw();    // Radians
            data.qw = imu->getQuatReal();
            data.qi = imu->getQuatI();
            data.qj = imu->getQuatJ();
            data.qk = imu->getQuatK();
        }

        if( report_ID == SENSOR_REPORTID_LINEAR_ACCELERATION ){
            data.ax = imu->getLinAccelX();
            data.ay = imu->getLinAccelY();
            data.az = imu->getLinAccelZ();
        }

        // Read Raw Gyro data
        if( report_ID == SENSOR_REPORTID_GYROSCOPE ) {
            data.gx = LPF( imu->getGyroX(), data.gx, 0.4 ); // Radians / second
            data.gy = LPF( imu->getGyroY(), data.gy, 0.4 ); // Radians / second
            data.gz = LPF( imu->getGyroZ(), data.gz, 0.4 ); // Radians / second
        }

        data.status.imu = 1;

    }
}

void Sensors::sample_flow(){

    int16_t dx, dy;
    float vx, vy;
    float dt = (float)(micros() - last_flow_sample)/1000000;

    // if (dt > 0.05) return;
    
    flow->readMotionCount( &dx, &dy );

    vx = (float)dx * data.z;
    vy = (float)dy * data.z;

    data.vx = LPF( vx, data.vx, 0.4 ); // meter / second
    data.vy = LPF( vy, data.vy, 0.4 ); // meter / second

    data.status.flow = 1;

}

void Sensors::sample_lidar(){

    if( lidar->dataReady() ){

        uint16_t value = lidar->read(false); // Non blocking

        // Altitude measured in body frame.
        float zb = ((float)value)/1000; // Convert to meters

        // Rotate altitude to world frame
        data.z = zb*cos( data.pitch)*cos(data.roll);

        data.status.lidar = 1;

    }

}

float Sensors::LPF( float new_sample, float old_sample, float alpha ){
    return ((alpha * new_sample) + (1.0-alpha) * old_sample);  
}