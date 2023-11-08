#include "RobotOdometry.h"
#include <math.h>

// Define the constant representing the distance between the robot's wheels
const float RobotOdometry::WHEEL_DISTANCE = 42.0;  // Replace with the actual value

// Constructor for the RobotOdometry class
RobotOdometry::RobotOdometry()
    : x(0), y(0), theta(0), traveledDistance(0), delta_distance(0), delta_theta(0), prevLeftEncoder(0), prevRightEncoder(0) {
  // Initialize the initial values of the odometry data
}

// Update the odometry data with new encoder values
void RobotOdometry::update(int leftEncoder, int rightEncoder) {
  // Calculate changes in encoder values
  int leftEncoderChange = leftEncoder - prevLeftEncoder;
  int rightEncoderChange = rightEncoder - prevRightEncoder;

  // Calculate distances traveled by each wheel using encoder values
  float leftDistance = (leftEncoderChange / 358.3) * 32 * M_PI;
  float rightDistance = (rightEncoderChange / 358.3) * 32 * M_PI;

  // Calculate changes in orientation and distance
  delta_theta = ((rightDistance - leftDistance) / (2 * WHEEL_DISTANCE));
  delta_distance = (leftDistance + rightDistance) / 2;

  // Update odometry data based on previous data
  theta += delta_theta;

  // Ensure that theta wraps around between -π and π
  while (theta > M_PI) {
    theta -= 2 * M_PI;
  }
  while (theta < -M_PI) {
    theta += 2 * M_PI;
  }

  // Update x and y positions using the updated odometry data
  x += delta_distance * cos(theta + (delta_theta / 2));
  y += delta_distance * sin(theta + (delta_theta / 2));

  // Update traveled distance
  traveledDistance += delta_distance;

  // Update previous encoder values for the next iteration
  prevLeftEncoder = leftEncoder;
  prevRightEncoder = rightEncoder;
}

// Getter methods to retrieve various odometry parameters
float RobotOdometry::getPositionX() const {
  return x;
}

float RobotOdometry::getPositionY() const {
  return y;
}

float RobotOdometry::getOrientationThetaRadians() const {
  return theta;
}

float RobotOdometry::getOrientationThetaDegrees() const {
  return (theta * 180.0 / M_PI);
}

float RobotOdometry::getTraveledDistance() const {
  return traveledDistance;
}

float RobotOdometry::getDeltaDistance() const {
  return delta_distance;
}

float RobotOdometry::getDeltaThetaRadians() const {
  return delta_theta;
}

float RobotOdometry::getDeltaThetaDegrees() const {
  return (delta_theta * 180.0 / M_PI);
}

// Reset the odometry data to its initial state
void RobotOdometry::reset() {
  x = 0;
  y = 0;
  theta = 0;
  traveledDistance = 0;
  delta_distance = 0;
  delta_theta = 0;
  prevLeftEncoder = 0;
  prevRightEncoder = 0;
}
