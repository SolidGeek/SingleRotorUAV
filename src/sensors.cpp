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
    // data.status = {};
    
    return temp;

}

void Sensors::run_estimator(){

    // Fill H and Z matrix with zeros, until measurements are ready
    H.Fill(0);
    Z.Fill(0);

    // Fill input vector with acceleration
    U << data.ax, data.ay, data.az;

    // Fill measurement vector with data
    if( data.status.flow == 1){
        H(3,3) = 1; H(4,4) = 1;
        Z(3) = data.vx;
        Z(4) = data.vy;
    }

    if( data.status.lidar == 1){
        H(2,2) = 1;
        Z(2) = data.z;
    }

    // Prediction, based on previous state and input
    Xpre = A*X + B*U; 

    // Update prediction with update using measurements 
    X = Xpre + Kf*(Z - H*Xpre); 
    
    // Fill estimate struct with values (for telemetry and stuff)
    estimate.x = X(0);
    estimate.y = X(1);
    estimate.z = X(2);
    estimate.vx = X(3);
    estimate.vy = X(4);
    estimate.vz = X(5);
}

void Sensors::sample_imu(){

    uint16_t report_ID;
    float accel[3]; 

    while ( report_ID = imu->getReadings() ) {
        // Read Attitude estimate
        if( report_ID == SENSOR_REPORTID_ROTATION_VECTOR ){
            data.roll   = imu->getRoll();   // Radians
            data.pitch  = imu->getPitch();  // Radians
            data.yaw    = imu->getYaw();    // Radians
        }

        // Linear acceleration is gravity componsated (but still measured in body frame)
        if( report_ID == SENSOR_REPORTID_LINEAR_ACCELERATION ){
            accel[0] = imu->getLinAccelX();
            accel[1] = imu->getLinAccelY();
            accel[2] = imu->getLinAccelZ();

            // We are only interested in acceleration in world frame
            rotate_to_world( accel );

            data.ax = LPF( accel[0], data.ax, 1 ) ; // Filter coefficient 1 = No lowpass
            data.ay = LPF( accel[1], data.ay, 1 ) ;
            data.az = LPF( accel[2], data.az, 0.8 ) ;
        }

        // Read Raw Gyro data
        if( report_ID == SENSOR_REPORTID_GYROSCOPE ) {
            data.gx = LPF( imu->getGyroX(), data.gx, 0.8 ); // Radians / second
            data.gy = LPF( imu->getGyroY(), data.gy, 0.8 ); // Radians / second
            data.gz = LPF( imu->getGyroZ(), data.gz, 0.8 ); // Radians / second
        }

        data.status.imu = 1;

    }
}

void Sensors::sample_flow()
{
    int16_t dx, dy;
    float vbx, vby;
    float vx, vy;
    float u = data.yaw;
    
    // Here dy and dx are flipped, to match orientation of IMU (which is placed to align with the body frame)
    flow->readMotionCount( &dy, &dx );

    // Convert flow to velocity and rotate to world frame (assumes flat surface)
    vbx = (float)dx * data.z * 0.5; // 0.5 = scaling factor found experimentally
    vby = (float)dy * data.z * 0.5;
    vx = cos(u) * vbx - sin(u) * vby;
    vy = sin(u) * vbx + cos(u) * vby;

    // Apply sligth lowpass filter in improve filter performance
    data.vx = LPF( vx, data.vx, 1 ); // meter / second
    data.vy = LPF( vy, data.vy, 1 ); // meter / second

    data.status.flow = 1;
}

void Sensors::sample_lidar(){

    float z, zb;

    if( lidar->dataReady() ){

        uint16_t value = lidar->read(false); // Non blocking

        // Altitude measured in body frame.
        zb = ((float)value)/1000 - SENSOR_LIDAR_OFFSET; // Convert to meters

        // Sanity check
        if(zb < 0 )
            zb = 0;

        // Rotate altitude to world frame
        z = zb * cos( data.pitch ) * cos( data.roll );

        data.z = LPF( z , data.z, 1 );

        data.status.lidar = 1;

    }

}

float Sensors::LPF( float new_sample, float old_sample, float alpha ){
    return ((alpha * new_sample) + (1.0-alpha) * old_sample);  
}

void Sensors::rotate_to_world( float * vector ){

    float p = data.roll;
    float q = data.pitch;
    float u = data.yaw;

    Matrix<3,1> in = { vector[0], vector[1], vector[2] };
    Matrix<3,1> out = {0,0,0};

    Matrix<3,3> R = {   cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ,
                        cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ,
                        -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                      };

    out = R * in;

    vector[0] = out(0);
    vector[1] = out(1);
    vector[2] = out(2);

}