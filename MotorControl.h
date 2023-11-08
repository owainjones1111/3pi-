#ifndef MotorControl_h
#define MotorControl_h

#include "Arduino.h"

// Define motor pin and direction constants
#define L_PWM 10  // Left motor PWM pin
#define L_DIR 16  // Left motor direction pin
#define R_PWM 9   // Right motor PWM pin
#define R_DIR 15  // Right motor direction pin
#define REV HIGH  // Reverse direction constant
#define FWD LOW   // Forward direction constant

// MotorControl class definition
class MotorControl {
  public:
    // Constructor to initialize the class with motor pin values
    MotorControl(int leftPwm, int leftDir, int rightPwm, int rightDir);

    // Method to set up the motors (usually called in the setup() function)
    void setupMotors();

    // Method to stop both motors
    void stopMotors();

    // Method to set the PWM values for both motors
    void setMotorsPWM(int leftPwmValue, int rightPwmValue);

    // Methods to control the movement of the robot
    void moveLeft(int pwmValue);     // Turn left with a specified PWM value
    void moveRight(int pwmValue);    // Turn right with a specified PWM value
    void moveForward(int pwmValue);  // Move forward with a specified PWM value
    void moveBackward(int pwmValue); // Move backward with a specified PWM value

  private:
    int _leftPwm;   // Store the left motor PWM pin
    int _leftDir;   // Store the left motor direction pin
    int _rightPwm;  // Store the right motor PWM pin
    int _rightDir;  // Store the right motor direction pin
};
#endif
