//// ROS includes
#include <ros.h>
#include <std_msgs/Int32.h>
#include <SoftwareServo.h>

// IMU includes
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// Motor driver board includes
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// general includes
#include <StandardCplusplus.h>
#include <vector>
#include <Arduino.h>
#include <math.h>
#include <TimerOne.h>


// project includes
#include "tread_controller.h"
#include "arm_controller.h"

// globals
Adafruit_BNO055 bno = Adafruit_BNO055(55); // sensor object declaration
float rotateAngle = 0.0; // the angle by which the robot should turn

float distanceToTravel = 0.0; // the commanded distance

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

String incomingString; // to store incoming value from serial monitor
const short buffer_length = 10; // length of incoming value
char check[buffer_length] = {0}; // to store the incoming value in an array
char incoming_array[buffer_length - 1] = {0}; // store the check[] array without the first element

ros::NodeHandle nh;
Tread_Controller tread_controller;
Arm_Controller arm_controller(5, 6, 7); // pins for roll, pitch, grasp; values between 0-180
std_msgs::Int32 us_dist; // Duration used to calculate distance
ros::Publisher pub_ultrasonic("/real/ultrasonic", &us_dist);

// namespaces
using namespace std;
using namespace imu;

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
*/
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

void imu_init() {
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // Load the calibration data from EEPROM
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
     Look for the sensor's unique ID at the beginning oF EEPROM.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(200);
  }
  else
  {
    Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }

  delay(1000);
  bno.setExtCrystalUse(true); // use of external crystal
  delay(1000);

}

void setup() {

  // general init
  nh.initNode();
  Serial.begin(115200);
  // Adafruit Motor Shield initialization
  AFMS.begin();  // create with the default frequency 1.6KHz
  nh.advertise(pub_ultrasonic); // to test connection
  pinMode(13, OUTPUT);

  // initialise the Adafruit IMU
  imu_init();

  servo refresh interrupt
  Timer1.initialize(50000);
  Timer1.attachInterrupt(servoRefreshInterrupt);
  Timer1.start();

}

// BUGS
//   1 . robot can't lift the pitch of the arm up because of low servo torque
void loop() {

  //  nh.spinOnce();
  if (Serial.available() > 0) {
    incomingString = Serial.readString(); // reads incoming string
    incomingString.toCharArray(check, buffer_length); // convert input string to char array

    // store only the numbers from the input array
    for (int i = 0; i < buffer_length - 1; i++) {
      incoming_array[i] = check[i + 1];
    }

    // check if forward motion
    if (check[0] == 'f') {
      distanceToTravel = atof(incoming_array);
      Serial.println("Movement detected");
      delay(10);
      Serial.println(distanceToTravel);
      delay(10);
      tread_controller.robot_drive(distanceToTravel);
    }

    // check if CW rotation
    if (check[0] == 'r') {
      rotateAngle = atof(incoming_array);
      Serial.println("Rotation detected");
      delay(10);
      tread_controller.robot_rotate_CW(rotateAngle);
    }

    // check if CCW rotation
    if (check[0] == 'l') {
      rotateAngle = atof(incoming_array);
      Serial.println("Rotation detected");
      delay(10);
      tread_controller.robot_rotate_CCW(rotateAngle);
    }
  }

  nh.spinOnce();
  arm_controller.change_pitch(ARM_UP);
  delay(2000);
  arm_controller.change_pitch(ARM_DOWN);
  delay(2000);
}

void servoRefreshInterrupt() {
  SoftwareServo::refresh();
}
