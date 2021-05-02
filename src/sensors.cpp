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

    /* ---- Sensor processing ---- */
    float p[3] = {0}; // Position vector (z body to world)
    float v[3] = {0}; // Velocity vector (vx, vy to world)
    float a[3] = {0}; // Acceleration vector (ax, ay, az to world)

    // Rotate lidar measurement to world frame
    p[2] = data.z;
    rotate_to_world( p );

    // Perform gyrocompensation on flow and rotate to world frame.
    v[0] = data.vx * p[2] - data.gy * p[2];
    v[1] = data.vy * p[2] + data.gx * p[2];
    rotate_to_world( v );

    // Rotate acceleration to world frame
    rotate_to_world( a );


    /* ---- Estimation ---- */
    // Fill input vector with acceleration
    H.Fill(0);
    Z.Fill(0);
    U << a[0], a[1], a[2];

    // Fill measurement vector with data
    if( stat_pos_x == 1 ){
        H(0,0) = 1;
        Z(0) = vicon_pos_x;
    }
    if( stat_pos_y == 1 ){
        H(1,1) = 1;
        Z(1) = vicon_pos_y;
    }

    if( data.status.lidar == 1){
        H(2,2) = 1;
        Z(2) = p[2]; // p[2]: z
    }

    if( data.status.flow == 1){
        H(3,3) = 1; H(4,4) = 1;
        Z(3) = v[0]; // vx
        Z(4) = v[1]; // vy
    }

    // Prediction, based on previous state and current input
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

    // Reset status (measurements has been used!)
    data.status.flow = 0;
    data.status.lidar = 0;
    data.status.imu = 0;
    stat_pos_x = false;
    stat_pos_y = false;
}

void Sensors::sample_imu(){

    uint16_t report_ID;
    float accel[3]; 

    while ( report_ID = imu->getReadings() ) {
        // Read Attitude estimate
        if( report_ID == SENSOR_REPORTID_ROTATION_VECTOR ){
            data.roll   = imu->getRoll();   // Radians
            data.pitch  = imu->getPitch();  // Radians
            yaw_raw     = imu->getYaw();    // Radians
            data.yaw    = rotate_yaw( yaw_raw );  
        }

        // Linear acceleration is gravity componsated (but still measured in body frame)
        if( report_ID == SENSOR_REPORTID_LINEAR_ACCELERATION ){
            accel[0] = imu->getLinAccelX();
            accel[1] = imu->getLinAccelY();
            accel[2] = imu->getLinAccelZ();

            data.ax = LPF( accel[0], data.ax, 1 ) ; // Filter coefficient 1 = No lowpass
            data.ay = LPF( accel[1], data.ay, 1 ) ;
            data.az = LPF( accel[2], data.az, 0.8 );
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
    static uint32_t last_sample;
    int16_t dx, dy;
    float ofx, ofy;
    float u = data.yaw;

    float dt = (micros() - last_sample);
    dt = dt/1000000;
    
    last_sample = micros();
    
    // Here dy and dx are flipped, to match orientation of IMU (which is placed to align with the body frame)
    flow->readMotionCount( &dy, &dx );

    // Convert change in pixels to unitless velocity 1/s
    ofx = ((float)dx / dt ) * 0.0022059; // scaling factor found experimentally (could probably be expressed using )
    ofy = ((float)dy / dt ) * 0.0022059;

    // Return the unitless velocity, which can be scaled by height
    data.vx = ofx; // pixels / second
    data.vy = ofy; // pixels / second

    data.status.flow = 1;
}

void Sensors::sample_lidar(){

    float zb;

    if( lidar->dataReady() ){

        uint16_t value = lidar->read(false); // Non blocking

        // Altitude measured in body frame.
        zb = ((float)value)/1000 - SENSOR_LIDAR_OFFSET; // Convert to meters

        // Sanity check
        if(zb < 0 )
            zb = 0;

        // Rotate altitude to world frame
        // z = zb * cos( data.pitch ) * cos( data.roll );

        data.z = LPF( zb , data.z, 1 );

        data.status.lidar = 1;

    }

}

float Sensors::LPF( float new_sample, float old_sample, float alpha ){
    return ((alpha * new_sample) + (1.0-alpha) * old_sample);  
}



void Sensors::set_origin(){
    yaw_origin = yaw_raw;

    X.Fill(0);
}


void Sensors::update_pos_x( float x ){
    stat_pos_x = true;

    vicon_pos_x = x;
}

void Sensors::update_pos_y( float y ){
    stat_pos_y = true;

    vicon_pos_y = y;
}

// Rotate yaw to align with origin / home
float Sensors::rotate_yaw( float yaw ){

    float rotated = yaw - yaw_origin;

    if( rotated > PI )
        rotated -= TWO_PI;
    else if( rotated < -PI )
        rotated += TWO_PI;
    
    return rotated;

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