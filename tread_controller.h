#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

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
#include <math.h>


class Tread_Controller {

    Adafruit_BNO055 bno; // sensor object declaration
    // Create the motor shield object with the default I2C address
    Adafruit_MotorShield AFMS = Adafruit_MotorShield();
    // Or, create it with a different I2C address (say for stacking)
    // Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

    // Select which 'port' M1, M2, M3 or M4. In this case, M1 and M4
    Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
    Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

    std::pair<float, float> initialVelocity; // a pair that stores initial zero velocities
    std::pair<float, float> velocities = {initialVelocity}; // a pair to store the final velocities after each function call to calc_dist()
    float ux = initialVelocity.first; // initial velocity in x-axis
    float uy = initialVelocity.second; // initial velocity in y-axis
    float vx = initialVelocity.first; // final velocity in x-axis
    float vy = initialVelocity.second; // final velocity in y-axis

    unsigned long current_Millis = 0; // store the milliseconds before the calculations for the distance begins
    float totalDistance = 0.0; // the distance the robot has moved
    float distTolerance = 0.008; // the tolerance for the distance the robot has moved
    unsigned long previousMillis = 0; // to store the old milliseconds and update the current_Millis
    float interval = 50.0;           //in ms

    float yaw = 0.00; // to store first rotation value
    float rotationTolerance = 5.00; // rotationTolerance is the tolerance angle
    float curr_yaw = 0.0; // rotation value for every measure
    int final_yaw = 0; // the relative angle that the robot should turn after calculations
    //    float rotateAngle = 0.0; // the angle by which the robot should turn
    bool isFirstTime = true; // isFirstTime is set to collect yaw data once at the beginning

  public:
    Tread_Controller();
    void move_forward();
    void rotate_left();
    void rotate_right();
    void move_backwards();
    void halt();
    float calc_dist();
    void robot_drive(float distanceToTravel);
    void robot_rotate_CW(float rotateAngle);
    void robot_rotate_CCW(float rotateAngle);

};

#endif
