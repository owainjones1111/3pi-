#include "MotorControl.h"

// Constructor to initialize the class with motor pin values
MotorControl::MotorControl(int leftPwm, int leftDir, int rightPwm, int rightDir) {
  _leftPwm = leftPwm;
  _leftDir = leftDir;
  _rightPwm = rightPwm;
  _rightDir = rightDir;
}

// Method to set up the motors (usually called in the setup() function)
void MotorControl::setupMotors() {
  // Set the motor pins as outputs
  pinMode(_leftPwm, OUTPUT);
  pinMode(_leftDir, OUTPUT);
  pinMode(_rightPwm, OUTPUT);
  pinMode(_rightDir, OUTPUT);

  // Initially, set the motor pins to LOW to stop the motors
  digitalWrite(_leftPwm, LOW);
  digitalWrite(_leftDir, LOW);
  digitalWrite(_rightPwm, LOW);
  digitalWrite(_rightDir, LOW);
}

// Method to stop both motors by setting PWM to 0
void MotorControl::stopMotors() {
  analogWrite(_leftPwm, 0);
  analogWrite(_rightPwm, 0);
}

// Method to set the PWM values for both motors
void MotorControl::setMotorsPWM(int leftPwmValue, int rightPwmValue) {
  // Set the left motor direction based on the PWM value's sign
  if (leftPwmValue < 0) {
    digitalWrite(_leftDir, REV);  // Set the direction to reverse
    leftPwmValue = -leftPwmValue; // Make PWM value positive
  } else {
    digitalWrite(_leftDir, FWD);  // Set the direction to forward
  }

  // Set the right motor direction based on the PWM value's sign
  if (rightPwmValue < 0) {
    digitalWrite(_rightDir, REV);   // Set the direction to reverse
    rightPwmValue = -rightPwmValue; // Make PWM value positive
  } else {
    digitalWrite(_rightDir, FWD);   // Set the direction to forward
  }

  // Set the PWM values for both motors
  analogWrite(_leftPwm, leftPwmValue);
  analogWrite(_rightPwm, rightPwmValue);
}

// Methods to control the movement of the robot
void MotorControl::moveLeft(int pwmValue) {
  setMotorsPWM(-pwmValue, pwmValue); // Turn left by setting PWM values
}

void MotorControl::moveRight(int pwmValue) {
  setMotorsPWM(pwmValue, -pwmValue); // Turn right by setting PWM values
}

void MotorControl::moveForward(int pwmValue) {
  setMotorsPWM(pwmValue, pwmValue); // Move forward by setting PWM values
}

void MotorControl::moveBackward(int pwmValue) {
  setMotorsPWM(-pwmValue, -pwmValue); // Move backward by setting PWM values
}
