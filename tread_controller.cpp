#include "tread_controller.h"

Tread_Controller::Tread_Controller() {

}

void Tread_Controller::move_forward() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void Tread_Controller::rotate_left() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
}

void Tread_Controller::rotate_right() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}

void Tread_Controller::move_backwards() {
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void Tread_Controller::halt() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

/**************************************************************************/
/*
    Calculate the intermediate distance using accelerometer values
*/
/**************************************************************************/
float Tread_Controller::calc_dist() {
  bno = Adafruit_BNO055(55);
  initialVelocity.first = 0.0;
  initialVelocity.second = 0.0;
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);    //in m/s2

  float ax = acc.x(); // in meters/second^2
  float ay = acc.y(); // in meters/second^2

  ux = velocities.first; // in meters/second
  uy = velocities.second; // in meters/second

  // calculate the change in velocities in x and y axes
  // velocity for the next timestep
  vx = ux + (ax * (interval * 0.001)); // in meters/second
  vy = uy + (ay * (interval * 0.001)); // in meters/second

  // find the distance in x and y axes
  float sx = (ux * (interval * 0.001)) + (0.5 * ax * (pow((interval * 0.001), 2)));
  float sy = (uy * (interval * 0.001)) + (0.5 * ay * (pow((interval * 0.001), 2)));

  float s = sqrt(pow(sx, 2) + pow(sy, 2));

  // store the final velocities to update the initial velocities for next timestep
  velocities.first = vx;
  velocities.second = vy;

  return s; // in meters
}


/**************************************************************************/
/*
    Forward drive distance accumulation
*/
/**************************************************************************/
void Tread_Controller::robot_drive(float distanceToTravel) {
  //      current_Millis = millis();

  while ((distanceToTravel - totalDistance) > distTolerance) {

    current_Millis = millis();
    // For timing the measurements for correct interval
    if (current_Millis - previousMillis > interval) {
      move_forward();
      Serial.print("td: "); Serial.println(totalDistance);
      delay(10);
      totalDistance += calc_dist();
      previousMillis = current_Millis;
      Serial.print("previousMillis: "); Serial.println(previousMillis);
      delay(10);
    }
  }
  halt(); // stop the robot
  Serial.println("Reached");
  totalDistance = 0.0; // reset the interative distance variable
}


/**************************************************************************/
/*
    Clockwise roatation of robot until specified angle
*/
/**************************************************************************/
void Tread_Controller::robot_rotate_CW(float rotateAngle) {
  if (isFirstTime == true) {
    imu::Vector<3> eulerAngle = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw = eulerAngle.x(); // store first degree in variable yaw
    isFirstTime = false;
  }

  // calculate the relative angle to rotate
  final_yaw = (((int)yaw + (int)rotateAngle) % 360);

  while (curr_yaw > ( (float)final_yaw + rotationTolerance / 2.0 ) ||
         curr_yaw < ( (float)final_yaw - rotationTolerance / 2.0 )) {
    rotate_right(); // rotate right
    imu::Vector<3> eulerAngle = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    curr_yaw = eulerAngle.x(); // store current degrees every timestep

    Serial.print(curr_yaw);
    Serial.print(", ");
    Serial.print(final_yaw);
    Serial.print(", ");
    Serial.println(rotationTolerance);
    delay(10);
  }
  halt(); // stop the robot
  //yaw = curr_yaw;
  imu::Vector<3> eulerAngle = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yaw = eulerAngle.x();
}


/**************************************************************************/
/*
    Counter-Clockwise roatation of robot until specified angle
*/
/**************************************************************************/
void Tread_Controller::robot_rotate_CCW(float rotateAngle) {
  if (isFirstTime == true) {
    imu::Vector<3> eulerAngle = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw = eulerAngle.x(); // store first degree in variable yaw
    isFirstTime = false;
  }

  // calculate the relative angle to rotate
  final_yaw = ((int)yaw - (int)rotateAngle);
  Serial.println("I'm here");
  delay(10);
  if (final_yaw < 0) {
    final_yaw = 360 + final_yaw;
  }
  curr_yaw = yaw;
  while (curr_yaw < ( (float)final_yaw - rotationTolerance / 2.0 ) ||
         curr_yaw > ( (float)final_yaw + rotationTolerance / 2.0 )) {
    rotate_left(); // rotate left
    imu::Vector<3> eulerAngle = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    curr_yaw = eulerAngle.x(); // store current degrees every timestep

    Serial.print(curr_yaw);
    Serial.print(", ");
    Serial.print(final_yaw);
    Serial.print(", ");
    Serial.println(rotationTolerance);
    delay(10);
  }
  halt(); // stop the robot
  imu::Vector<3> eulerAngle = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yaw = eulerAngle.x();
}

