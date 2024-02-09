/**
 * @ Author: Pallab Maji
 * @ Create Time: 2024-01-31 10:53:51
 * @ Modified time: 2024-02-08 17:27:53
 * @ Description: Arduino code to Read and Parse Sensor Outputs to ROS Node over ROS-Serial
 *              This code is to be run on the Arduino Mega 2560 board
 *              This code is to be used with the ROS Node: vehicle_dynamics
 *              The Sensors Interfaced are as follows:
 *              1. IMU: MPU6050
 *                    - Code is based on the example code from the MPU6050 Library from Jeff Rowberg <jeff@rowberg.net>
 *                    - The code is modified to use the DMP for orientation data and to use the FIFO buffer as available
 * at https://github.com/jrowberg/i2cdevlib
 *              2. GPS: NEO-7M
 *              3. Wheel SPeed Encoder: AS5048A
 *
 *
 *  TODO: 2. Add Encoder Data
 *  TODO: 3. Add GPS_Common/GPSFix Message from NavSatFix, and merge the time and Speed messgaes
 */

#include <Arduino.h>

#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
// #include <gps_common/GPSFix.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// GPS Libraries
#include "TinyGPS++.h"

// IMU Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// // TF Libraries
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>

#define SERIAL_BAUD 57600
#define GPS_BAUD 9600 // GPS is connected to Serial3 & NEO7m Module works at 9600 baud rate
#define GPS_SERIAL Serial3

// Define Sampling Times
#define GPS_Sampling_Time_ms 100
#define IMU_Sampling_Time_ms 100
#define ENCODER_Sampling_Time_ms 100

// Define PINS and other constants here:
#define IMU_LED 13
#define GPS_LED 12
#define ENCODER_LED 11
// Initialize Speed Sensor
#define SPEED_SENSOR_PIN 3

#define INTERRUPT_PIN 2 // use pin 2 on Arduino for MPU6050 interrupt
bool blinkState_IMU = false;

#define MESSAGE_FRAME_ID "vedyne_0"

#define LOGGER nh.loginfo // Define Logger as Serial or ROS_INFO: Vales: Serial.println or nh.loginfo

// Define Sensor Objects
TinyGPSPlus gps;
// TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
// TinyGPSCustom hdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
// TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element

// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu; // MPU6050 object; Use MPU6050 mpu(0x69) to use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 gyro;    // [x, y, z]            gyro sensor measurements
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
geometry_msgs::Quaternion orientation;
geometry_msgs::Quaternion differential_orientation;
geometry_msgs::Quaternion zero_orientation;
bool setzero_orientation = true;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[28] = {'$', 0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0,    0,    0,
                            0,   0,    0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// MPU interrupt pin configuration & detection Routine
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// Define ROS Node Handle
ros::NodeHandle nh;

// Define ROS Publishers
// gps_common::GPSFix gps_msg;
sensor_msgs::NavSatFix gps_msg;
ros::Publisher gps_pub("/vehicle_dynamics/gps", &gps_msg);
std_msgs::Float32 gps_speed_msg;
ros::Publisher gps_speed_pub("/vehicle_dynamics/gps_speed", &gps_speed_msg);
std_msgs::String gps_time_msg;
ros::Publisher gps_time_pub("/vehicle_dynamics/gps_time", &gps_time_msg);
sensor_msgs::Temperature imu_temp_msg;
ros::Publisher imu_temp_pub("/vehicle_dynamics/imu_temp", &imu_temp_msg);
std_msgs::Float32 encoder_speed_msg;
ros::Publisher encoder_speed_pub("/vehicle_dynamics/encoder_speed", &encoder_speed_msg);

// Define Global Variables
unsigned long currentMillis_GPS = 0;
unsigned long previousMillis_GPS = 0;

void get_gps_data(TinyGPSPlus &, sensor_msgs::NavSatFix &, std_msgs::Float32 &, std_msgs::String &);
// void get_gps_data(TinyGPSPlus &, gps_common::GPSFix &);
void get_gps_empty_msg(sensor_msgs::NavSatFix &, std_msgs::Float32 &, std_msgs::String &);
void copy_gps_msg(sensor_msgs::NavSatFix &, sensor_msgs::NavSatFix &, std_msgs::Float32 &, std_msgs::Float32 &,
                  std_msgs::String &, std_msgs::String &);
bool init_gps_modules(TinyGPSPlus &);

// Define IMU
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/vehicle_dynamics/imu", &imu_msg);

void get_imu_data(MPU6050 &, sensor_msgs::Imu &);
bool init_imu_modules(MPU6050 &);

// Define Encoder
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = false;
float prev_speed = 0;

void get_encoder_data(std_msgs::Float32 &);
void get_encoder_data_interrupt(std_msgs::Float32 &);
unsigned long pulseInLongCustom(uint8_t pin, uint8_t state);
bool init_speed_sensor();

//

// --------------------------------------------------------------------------------
//                     INITIAL SETUP
// --------------------------------------------------------------------------------

void setup()
{
    // put your setup code here, to run once:
    // Initialize Serial Communication
    Serial.begin(SERIAL_BAUD);
    GPS_SERIAL.begin(GPS_BAUD);

    // Initialize I2C Communication
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // Initialize ROS Node
    nh.initNode();
    nh.advertise(gps_pub);
    nh.advertise(gps_speed_pub);
    nh.advertise(gps_time_pub);
    nh.advertise(imu_pub);
    nh.advertise(imu_temp_pub);
    nh.advertise(encoder_speed_pub);

    // Initialize LED Pins
    pinMode(IMU_LED, OUTPUT);
    pinMode(GPS_LED, OUTPUT);
    pinMode(ENCODER_LED, OUTPUT);
    pinMode(SPEED_SENSOR_PIN, INPUT);

    // ------------------------------
    // Initialize Sensor Objects
    // ------------------------------

    // Initialize GPS Module
    if (init_gps_modules(gps))
    {
        LOGGER("[INFO]: GPS ROS Node Initialized");
    }
    else
    {
        LOGGER("[INFO]: GPS ROS Node Initialization Failed");
    }

    // Initialize IMU Module
    if (init_imu_modules(mpu))
    {
        LOGGER("[INFO]: IMU ROS Node Initialized");
        // configure LED for output
        pinMode(IMU_LED, OUTPUT);
    }
    else
    {
        LOGGER("[INFO]: IMU ROS Node Initialization Failed");
    }

    // Initialize Speed Sensor
    if (init_speed_sensor())
    {
        LOGGER("[INFO]: Speed Sensor ROS Node Initialized");
    }
    else
    {
        LOGGER("[INFO]: Speed Sensor ROS Node Initialization Failed");
    }
}

// --------------------------------------------------------------------------------

// --------------------------------------------------------------------------------
//                     MAIN LOOP
// --------------------------------------------------------------------------------

void loop()
{
    // put your main code here, to run repeatedly:
    // Update the ROS Node
    nh.spinOnce();

    // Update the GPS Data
    get_gps_data(gps, gps_msg, gps_speed_msg, gps_time_msg);

    // Update the IMU Data
    get_imu_data(mpu, imu_msg);
    // LOGGER("IMU Data Updated");

    // Update the Encoder Data
    get_encoder_data_interrupt(encoder_speed_msg);
    //    get_encoder_data(encoder_speed_msg);

    // Blink the LED
    if (blinkState_IMU)
    {
        digitalWrite(IMU_LED, HIGH);
    }
    else
    {
        digitalWrite(IMU_LED, LOW);
    }
    blinkState_IMU = !blinkState_IMU;

    // Delay for the next Loop
    // delay(10);
}

// --------------------------------------------------------------------------------
//                     FUNCTION DEFINITIONS - Speed Encoder
// --------------------------------------------------------------------------------

void speed_sensor_interrupt()
{
    if (digitalRead(SPEED_SENSOR_PIN) == HIGH)
    {
        pulseInTimeBegin = micros();
    }
    else
    {
        pulseInTimeEnd = micros();
        newPulseDurationAvailable = true;
    }
}

bool init_speed_sensor()
{
    LOGGER("[INFO] Initializing Speed Sensor");

    // Attach the Speed Sensor as interrupt
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), speed_sensor_interrupt, CHANGE);
    newPulseDurationAvailable = true;

    return true;
}

void get_encoder_data_interrupt(std_msgs::Float32 &encoder_speed_msg)
{

    float speed_kmph = 0.0;
    if (newPulseDurationAvailable)
    {
        newPulseDurationAvailable = false;
        unsigned long pulseDuration = pulseInTimeEnd - pulseInTimeBegin;

        // encoder_speed_msg.data = pulseDuration;
        // encoder_speed_pub.publish(&encoder_speed_msg);

        if (pulseDuration > 100000 || pulseDuration <= 0)
        {
            speed_kmph = prev_speed;
            encoder_speed_msg.data = speed_kmph;
            encoder_speed_pub.publish(&encoder_speed_msg);
            return;
        }
        else
        {
            float frequency = 1000000.0 / pulseDuration;
            frequency = frequency / 1.4; // 0.065 is the radius of the wheel
            speed_kmph = frequency * (20.0 / 34.0);
            if (speed_kmph > 200 && speed_kmph < 1)
                return;
            encoder_speed_msg.data = speed_kmph;
            encoder_speed_pub.publish(&encoder_speed_msg);
            prev_speed = speed_kmph;
        }
    }
    else
    {
        LOGGER("[WARNING] Not Getting Encoder");
        encoder_speed_msg.data = prev_speed;
        encoder_speed_pub.publish(&encoder_speed_msg);
    }
}

void get_encoder_data(std_msgs::Float32 &encoder_speed_msg)
{
    unsigned long timeout = 1 * 1000000;
    // Read the Speed Sensor
    //     unsigned long pulseDuration = pulseInLongCustom(SPEED_SENSOR_PIN, LOW);
    unsigned long pulseDuration = pulseInLong(SPEED_SENSOR_PIN, LOW, timeout);

    //    encoder_speed_msg.data = pulseDuration;
    //    encoder_speed_pub.publish(&encoder_speed_msg);

    if (pulseDuration == 0)
    {
        encoder_speed_msg.data = 0;
        encoder_speed_pub.publish(&encoder_speed_msg);
        return;
    }
    else
    {
        float frequency = 1000000.0 / pulseDuration;
        frequency = frequency / 2.6; // 0.065 is the radius of the wheel
        float speed_kmph = frequency * (20.0 / 34.0);
        encoder_speed_msg.data = speed_kmph;
        encoder_speed_pub.publish(&encoder_speed_msg);
    }
}

unsigned long pulseInLongCustom(uint8_t pin, uint8_t state)
{
    // cache the port and bit of the pin in order to speed up the
    // pulse width measuring loop and achieve finer resolution.  calling
    // digitalRead() instead yields much coarser resolution.
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    uint8_t stateMask = (state ? bit : 0);

    unsigned long startMicros = micros();

    // wait for any previous pulse to end
    while ((*portInputRegister(port) & bit) == stateMask)
    {
        // Serial.println("0 km/h");
        // Serial.println((*portInputRegister(port) & bit));
        // Serial.println(stateMask);
        // if (micros() - startMicros >= timeout){
        // Serial.println("Here1.1");
        // return 0;
        //}
    }

    // wait for the pulse to start
    while ((*portInputRegister(port) & bit) != stateMask)
    {
        // Serial.println("0 km/h");
        // Serial.println("Here2");
        // if (micros() - startMicros >= timeout){
        //  Serial.println("Here2.2");
        // return 0;
        // }
    }

    unsigned long start = micros();
    // wait for the pulse to stop
    while ((*portInputRegister(port) & bit) == stateMask)
    {
        // Serial.println("0 km/h");
        // Serial.println("Here3");
        // if (micros() - startMicros >= timeout){
        // Serial.println("Here3.3");
        // return 0;
        //}
    }
    return micros() - start;
}

// --------------------------------------------------------------------------------
//                     FUNCTION DEFINITIONS - GPS TinyGPS++
// --------------------------------------------------------------------------------
bool init_gps_modules(TinyGPSPlus &gps)
{
    LOGGER("[INFO] Initializing GPS Module");

    // Check for GPS Module Connection and Fix
    unsigned long start_time = millis();

    while (GPS_SERIAL.available() > 0)
    {
        if (gps.encode(GPS_SERIAL.read()))
        {
            LOGGER("[INFO] GPS Module Connection Successful");
            return true;
        }
        else if (millis() - start_time > 10000)
        {
            LOGGER("[INFO] GPS Module Connection Failed");
            return false;
        }
        else
        {
            continue;
        }
    }

    return true;
}

void get_gps_data(TinyGPSPlus &gps, sensor_msgs::NavSatFix &gps_msg, std_msgs::Float32 &gps_speed_msg,
                  std_msgs::String &gps_time_msg)
{
    // Check for GPS Data
    while (GPS_SERIAL.available() > 0)
    {
        if (gps.encode(GPS_SERIAL.read()))
        {
            // Update GPS Data
            gps_msg.header.stamp = nh.now();
            gps_msg.header.frame_id = MESSAGE_FRAME_ID;

            gps_msg.latitude = gps.location.lat();
            gps_msg.longitude = gps.location.lng();
            gps_msg.altitude = gps.altitude.meters();
            gps_msg.position_covariance[0] = gps.hdop.hdop();
            gps_msg.position_covariance[4] = gps.hdop.hdop();

            gps_speed_msg.data = gps.speed.kmph();
            char date_time_str[20];
            sprintf(date_time_str, "%02d/%02d/%02d %02d:%02d:%02d", gps.date.day(), gps.date.month(), gps.date.year(),
                    gps.time.hour(), gps.time.minute(), gps.time.second());

            gps_time_msg.data = date_time_str;

            // Publish GPS Data
            gps_pub.publish(&gps_msg);
            gps_speed_pub.publish(&gps_speed_msg);
            gps_time_pub.publish(&gps_time_msg);

            // LOGGER("[INFO] GPS Data Updated");
        }
    }
}

// --------------------------------------------------------------------------------
//                     FUNCTION DEFINITIONS - IMU MPU6050
// --------------------------------------------------------------------------------

bool init_imu_modules(MPU6050 &mpu)
{

    LOGGER("[INFO]: Initializing IMU Module");
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    LOGGER("[INFO]: Testing IMU Module Connection");
    LOGGER(mpu.testConnection() ? "[INFO]: IMU Module Connection Successful" : "[INFO]: IMU Module Connection Failed");

    // // wait for ready
    // LOGGER(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    LOGGER("[INFO]: Initializing DMP");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // Calibration Routine for MPU6050 is avaiable at Arduino imu_zero sketch
    mpu.setXAccelOffset(-1169);
    mpu.setYAccelOffset(744);
    mpu.setZAccelOffset(1620);
    mpu.setXGyroOffset(48);
    mpu.setYGyroOffset(47);
    mpu.setZGyroOffset(-8);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        // turn on the DMP, now that it's ready
        LOGGER(("[INFO] Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        LOGGER(("[INFO] Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        LOGGER(("[INFO] DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        return true;
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        LOGGER(("[ERROR] DMP Initialization failed (code "));
        // nh.logerror(devStatus);
        LOGGER(String(devStatus).c_str());
        return false;
    }
}

void get_imu_data(MPU6050 &mpu, sensor_msgs::Imu &imu_msg)
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //        LOGGER(("[WARNING] FIFO overflow!"));
        return;
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        // gyro values
        teapotPacket[10] = fifoBuffer[16];
        teapotPacket[11] = fifoBuffer[17];
        teapotPacket[12] = fifoBuffer[20];
        teapotPacket[13] = fifoBuffer[21];
        teapotPacket[14] = fifoBuffer[24];
        teapotPacket[15] = fifoBuffer[25];
        // accelerometer values
        teapotPacket[16] = fifoBuffer[28];
        teapotPacket[17] = fifoBuffer[29];
        teapotPacket[18] = fifoBuffer[32];
        teapotPacket[19] = fifoBuffer[33];
        teapotPacket[20] = fifoBuffer[36];
        teapotPacket[21] = fifoBuffer[37];
        // temperature
        int16_t temperature = mpu.getTemperature();
        teapotPacket[22] = temperature >> 8;
        teapotPacket[23] = temperature & 0xFF;
        // Serial.write(teapotPacket, 28);
        teapotPacket[25]++; // packetCount, loops at 0xFF on purpose

        // Get Quartenion values
        int data_packet_start = 0;
        int16_t w = (((0xff & (char)teapotPacket[data_packet_start + 2]) << 8) |
                     0xff & (char)teapotPacket[data_packet_start + 3]);
        int16_t x = (((0xff & (char)teapotPacket[data_packet_start + 4]) << 8) |
                     0xff & (char)teapotPacket[data_packet_start + 5]);
        int16_t y = (((0xff & (char)teapotPacket[data_packet_start + 6]) << 8) |
                     0xff & (char)teapotPacket[data_packet_start + 7]);
        int16_t z = (((0xff & (char)teapotPacket[data_packet_start + 8]) << 8) |
                     0xff & (char)teapotPacket[data_packet_start + 9]);

        double wf = w / 16384.0;
        double xf = x / 16384.0;
        double yf = y / 16384.0;
        double zf = z / 16384.0;

        geometry_msgs::Quaternion current_orientation;
        current_orientation.x = xf;
        current_orientation.y = yf;
        current_orientation.z = zf;
        current_orientation.w = wf;

        if (setzero_orientation)
        {
            zero_orientation.w = wf;
            zero_orientation.x = xf;
            zero_orientation.y = yf;
            zero_orientation.z = zf;

            setzero_orientation = false;
        }

        differential_orientation.x = wf * zero_orientation.x - xf * zero_orientation.w - yf * zero_orientation.z +
                                    zf * zero_orientation.y;
        differential_orientation.y = wf * zero_orientation.y + xf * zero_orientation.z - yf * zero_orientation.w -
                                    zf * zero_orientation.x;
        differential_orientation.z = wf * zero_orientation.z - xf * zero_orientation.y + yf * zero_orientation.x -
                                    zf * zero_orientation.w;
        differential_orientation.w = wf * zero_orientation.w + xf * zero_orientation.x + yf * zero_orientation.y +
                                    zf * zero_orientation.z;
                                    

        // Get Gyro Values
        int16_t gx = (((0xff & (char)teapotPacket[data_packet_start + 10]) << 8) |
                      0xff & (char)teapotPacket[data_packet_start + 11]);
        int16_t gy = (((0xff & (char)teapotPacket[data_packet_start + 12]) << 8) |
                      0xff & (char)teapotPacket[data_packet_start + 13]);
        int16_t gz = (((0xff & (char)teapotPacket[data_packet_start + 14]) << 8) |
                      0xff & (char)teapotPacket[data_packet_start + 15]);
        // calculate rotational velocities in rad/s
        // without the last factor the velocities were too small
        // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
        // FIFO frequency 100 Hz -> factor 10 ?
        // seems 25 is the right factor
        // TODO: check / test if rotational velocities are correct
        double gxf = gx * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;
        double gyf = gy * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;
        double gzf = gz * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;

        // get acelerometer values
        int16_t ax = (((0xff & (char)teapotPacket[data_packet_start + 16]) << 8) |
                      0xff & (char)teapotPacket[data_packet_start + 17]);
        int16_t ay = (((0xff & (char)teapotPacket[data_packet_start + 18]) << 8) |
                      0xff & (char)teapotPacket[data_packet_start + 19]);
        int16_t az = (((0xff & (char)teapotPacket[data_packet_start + 20]) << 8) |
                      0xff & (char)teapotPacket[data_packet_start + 21]);
        // calculate accelerations in m/s²
        double axf = ax * (8.0 / 65536.0) * 9.81;
        double ayf = ay * (8.0 / 65536.0) * 9.81;
        double azf = az * (8.0 / 65536.0) * 9.81;

        // Update IMU Message Data

        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = MESSAGE_FRAME_ID;

        imu_msg.orientation.x = differential_orientation.x;
        imu_msg.orientation.y = differential_orientation.y;
        imu_msg.orientation.z = differential_orientation.z;
        imu_msg.orientation.w = differential_orientation.w;

        // imu_msg.orientation.x = xf;
        // imu_msg.orientation.y = yf;
        // imu_msg.orientation.z = zf;
        // imu_msg.orientation.w = wf;

        imu_msg.angular_velocity.x = gxf;
        imu_msg.angular_velocity.y = gyf;
        imu_msg.angular_velocity.z = gzf;

        imu_msg.linear_acceleration.x = axf;
        imu_msg.linear_acceleration.y = ayf;
        imu_msg.linear_acceleration.z = azf;

        
        // Print Temerature
        double temp = mpu.getTemperature() / 340.00 + 36.53;
        // LOGGER(("[INFO] IMU Temperature: "));
        // LOGGER((String(temp).c_str()));

        imu_temp_msg.temperature = temp;
        imu_temp_msg.header.stamp = nh.now();
        imu_temp_msg.header.frame_id = MESSAGE_FRAME_ID;

        // Publish IMU Data
        imu_pub.publish(&imu_msg);
        imu_temp_pub.publish(&imu_temp_msg);

        // LOGGER("[INFO}] IMU Data Updated");

    } // END: if (mpuIntStatus & 0x02)
} // END: get_imu_data

// --------------------------------END---------------------------------
