#include "LineSensor.h"
#include "MotorControl.h"
#include "RobotOdometry.h"
#include "Encoders.h"

# define BUZZER_PIN 6
// Function prototypes
void calibrateSensors(LineSensor& lineSensor, int minValues[], int maxValues[]);
void handleButtonPress(int buttonPin, bool& buttonState, unsigned long& lastButtonStateChange, void (*action)());
bool isRobotOnLine(LineSensor& lineSensor, int threshold[]);
void handleLineFollowing(LineSensor& lineSensor, MotorControl& motor, int threshold[], RobotOdometry& odometer, bool isCurrentlyOnLine);
void printSensorValues(LineSensor& lineSensor, bool isCurrentlyOnLine);
int r = 500;
const int ls_pins[NUM_SENSORS] = {DN1, DN2, DN3, DN4, DN5};
const unsigned long sensorReadInterval = 200;
int countGap = 0;
RobotOdometry odometer;
MotorControl motor(L_PWM, L_DIR, R_PWM, R_DIR);
LineSensor lineSensor(EMIT, ls_pins, NUM_SENSORS);

int minValues[NUM_SENSORS], maxValues[NUM_SENSORS], threshold[NUM_SENSORS];
long encoder0Count = 0;
long encoder1Count = 0;

const int buttonAPin = 14;  // Button A pin
const int buttonBPin = 30;  // Button B pin
bool isCalibrated = false;
bool buttonAPressed = false;
bool buttonBPressed = false;
bool startFSM = false;
bool exitBox = false;
bool startLogicFlag = false;
bool atGap = false;
bool gapFinish = false;
bool endLine = false;
bool atHome = false;
bool wasOnLine = false;  // Added variable to track previous line status
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
float start_x = 0;
float start_y = 0;
float pos_x = 0;
float pos_y = 0;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  delay(1000);  // Wait for system stability
  lineSensor.begin();  // Initialize line sensors
  lineSensor.setCalibrationValues(maxValues, minValues);
    setupEncoder0();
  setupEncoder1();
  motor.setupMotors();  // Set up motor control
  motor.stopMotors();  // Stop the motors
  Serial.println("DN1,DN2,DN3,DN4,DN5,W,Left PWM,Right PWM");  // Print a header
  pinMode(buttonAPin, INPUT_PULLUP);  // Set Button A as input with pull-up resistor
  pinMode(buttonBPin, INPUT_PULLUP);  // Set Button B as input with pull-up resistor
  isCalibrated = false;  // Initialize calibration flag
}
enum RobotState {
  IDLE,
  EXIT,
  FOLLOW_LINE,
  GAP,
  RETURN,
  STOP,
};
unsigned long getElapsedTime() {
  if (startTime == 0) {
    return 0; // Timer not started, elapsed time is zero
  } else {
    return millis() - startTime; // Calculate and return elapsed time
  }
}
RobotState currentState = IDLE;
void loop() {
  unsigned long lastButtonAStateChange = 0;
  unsigned long lastButtonBStateChange = 0;
  bool isCurrentlyOnLine = false;

  handleButtonPress(buttonAPin, buttonAPressed, lastButtonAStateChange, []() {
    if (!isCalibrated) {
      calibrateSensors(lineSensor, minValues, maxValues);
      isCalibrated = true;
    }
  });

  handleButtonPress(buttonBPin, buttonBPressed, lastButtonBStateChange, []() {
    if (isCalibrated) {
      //reportThresholdValues();
      startTime = 0;
      startTime = millis();
      odometer.reset();
      currentState = EXIT; // Set the flag to start the logic
      //startLogicFlag = true;
      }
  });
  elapsedTime = getElapsedTime();
  odometer.update(count_e0, count_e1);
  encoder0Count = count_e0;
  encoder1Count = count_e1;
  Serial.println(currentState);
  //reportValues();
  switch(currentState){
    case IDLE:{
      break;
    }
    case EXIT: {
    odometer.update(count_e0, count_e1);
    start_x = odometer.getPositionX();
    start_y = odometer.getPositionY();
    motor.moveForward(30);  // Adjust the forward speed as needed
    float currentD= odometer.getTraveledDistance();  // Record the current distance
    float targetD = currentD + 200.0;  // Calculate the target distance
    while (odometer.getTraveledDistance() < targetD) {
      // Wait until the robot has moved 20 cm
      odometer.update(count_e0, count_e1);
      Serial.println(odometer.getTraveledDistance());
    }
    motor.moveLeft(30);
    delay(200);
    motor.stopMotors();
    currentState = FOLLOW_LINE;
    break;
  }
  case FOLLOW_LINE: {
    isCurrentlyOnLine = isRobotOnLine(lineSensor, threshold);
    handleLineFollowing(lineSensor, motor, threshold, odometer, isCurrentlyOnLine);
    break;
  }
  case GAP: {
    motor.moveForward(30);  // Adjust the forward speed as needed
    float currentDistanceAtActivation = odometer.getTraveledDistance();  // Record the current distance
    float targetDistance = currentDistanceAtActivation + 200.0;  // Calculate the target distance

    while (odometer.getTraveledDistance() < targetDistance) {
      // Wait until the robot has moved 20 cm
      odometer.update(count_e0, count_e1);
      Serial.println(odometer.getTraveledDistance());
      currentState = FOLLOW_LINE;
    }
    break;
  }
  case RETURN: {
    // Calculate the current position
    pos_x = odometer.getPositionX();
    pos_y = odometer.getPositionY();

    // Calculate the distance to the starting point
    float distanceToStart = sqrt(pow(pos_x - start_x, 2) + pow(pos_y - start_y, 2));

    // If the robot is not at the starting point, navigate back
    if (distanceToStart > 5) {  // Adjust the threshold as needed
      // Calculate the angle to the starting point
      float angleToStart = atan2(start_y - pos_y, start_x - pos_x) * 180.0 / PI;

      // Adjust the robot's orientation to head towards the starting point
      float currentOrientation = odometer.getOrientationThetaDegrees();
      float orientationError = angleToStart - currentOrientation;

      if (orientationError < -180) {
        orientationError += 360;
      } else if (orientationError > 180) {
        orientationError -= 360;
      }

      if (fabs(orientationError) > 10) {  // Adjust the orientation error threshold
        // Rotate the robot to align with the starting point
        if (orientationError < 0) {
          motor.moveLeft(30);
        } else {
          motor.moveRight(30);
        }
      } else {
        // Move the robot forward towards the starting point
        motor.moveForward(30);  // Adjust the forward speed as needed
      }
    } else {
      // Stop the robot when it reaches the starting point
      motor.stopMotors();
      currentState = STOP;
    }
    break;
  }
  case STOP: {
  // Stop the motors
  motor.stopMotors();
  
  // Reset the odometry
  odometer.reset();
  atGap = false;
  // Transition back to the IDLE state
  currentState = IDLE;
  break;
}

  /*case RETURN: {
    pos_x = odometer.getPositionX();
    pos_y = odometer.getPositionY();
    /*Serial.print("x :");
    Serial.print(pos_x);
    Serial.print(" y :");
    Serial.println(pos_y);
    Serial.print(" start x :");
    Serial.print(start_x);
    Serial.print(" start y :");
    Serial.println(start_y);
    break;* /
}*/

  }
  
}

void calibrateSensors(LineSensor& lineSensor, int minValues[], int maxValues[]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = lineSensor.readSensor(i);
    maxValues[i] = lineSensor.readSensor(i);
  }
  Serial.println("Calibrating sensors...");

  for (int i = 0; i < 2000; i++) {
    if (i < 1000) {
      motor.setMotorsPWM(25, -25);
    } else {
      motor.setMotorsPWM(-25, 25);
    }

    for (int j = 0; j < NUM_SENSORS; j++) {
      float sensorValue = lineSensor.readSensor(j);
      minValues[j] = min(sensorValue, minValues[j]);
      maxValues[j] = max(sensorValue, maxValues[j]);
    }

    if (i % 20 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print(i / 20);
      Serial.println("%");
    }
  }
  Serial.println("Calibration completed.");
  Serial.print("Threshold values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
  motor.stopMotors();
  odometer.reset();
}

void handleButtonPress(int buttonPin, bool& buttonState, unsigned long& lastButtonStateChange, void (*action)()) {
  bool currentButtonState = digitalRead(buttonPin);
  if (currentButtonState != buttonState) {
    if (currentButtonState == LOW) {
      action();
    }
    buttonState = currentButtonState;
    lastButtonStateChange = millis();
  }
}


bool isRobotOnLine(LineSensor& lineSensor, int threshold[]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    float sensorValue = lineSensor.readSensor(i);
    if (sensorValue > threshold[i]) {
      return true;
    }
  }
  return false;
}

void handleLineFollowing(LineSensor& lineSensor, MotorControl& motor, int threshold[], RobotOdometry& odometer, bool isCurrentlyOnLine) {
  if (isCurrentlyOnLine != wasOnLine) {
    wasOnLine = isCurrentlyOnLine;
    printSensorValues(lineSensor, isCurrentlyOnLine);
  }

  int sensor0Value = lineSensor.readSensor(0);
  int sensor4Value = lineSensor.readSensor(4);

  if (sensor0Value > threshold[0] || sensor4Value > threshold[4]) {
    // Outer sensors triggered, move forward for 0.5 cm
    motor.moveForward(30);  // Adjust the forward speed as needed
    float initialDistance = odometer.getTraveledDistance();
    while (odometer.getTraveledDistance() - initialDistance < 15) {
      // Wait until the robot has moved 0.5 cm
      odometer.update(count_e0, count_e1);
    }
    motor.stopMotors();

    // Then, make an extreme turn
    if (sensor0Value > sensor4Value) {
      // Turn left more if sensor 0 is triggered
      motor.moveLeft(50);  // Increase the turning speed
      buzzerTone(1000, 200);  // Play a short buzzer sound
    } else {
      // Turn right more if sensor 4 is triggered
      motor.moveRight(50);  // Increase the turning speed
      buzzerTone(1000, 200);  // Play a short buzzer sound
    }
  } else if (lineSensor.readSensor(2) > threshold[2]) {
    // Center sensor is on the line, move forward
    motor.moveForward(30);  // Adjust the forward speed as needed
  } 
  else if (lineSensor.normalizeAndMapSensorValue(1, 0, 100) < 20 && isCurrentlyOnLine) {
    motor.moveRight(30);
  } else if (lineSensor.normalizeAndMapSensorValue(3, 0, 100) < 20 && isCurrentlyOnLine) {
    motor.moveLeft(30);
  } else if(!isCurrentlyOnLine) {
    if(elapsedTime <7500 ){
      turn180Degrees(odometer, motor, lineSensor, threshold);

    }
    else if(currentState != GAP && !atGap && elapsedTime > 8000){
      atGap = true;
      currentState = GAP;
    }
    else if(elapsedTime>20000){
    motor.stopMotors();
    float distanceTraveled = odometer.getTraveledDistance();
    currentState = RETURN;
    Serial.print("Distance Traveled: ");
    Serial.print(odometer.getOrientationThetaDegrees());
    Serial.println(" mm");}
    else{
    motor.stopMotors();
    float distanceTraveled = odometer.getTraveledDistance();
    Serial.print("Distance Traveled: ");
    Serial.print(odometer.getOrientationThetaDegrees());
    Serial.println(" mm");}
  }
}

void buzzerTone(int frequency, int duration) {
  int period = 1000000L / frequency;  // Calculate the period in microseconds
  int halfPeriod = period / 2;  // Calculate half of the period

  for (long elapsed = 0; elapsed < duration * 1000L; elapsed += period) {
    digitalWrite(BUZZER_PIN, HIGH);  // Turn the buzzer on
    delayMicroseconds(halfPeriod);   // Wait for half the period
    digitalWrite(BUZZER_PIN, LOW);   // Turn the buzzer off
    delayMicroseconds(halfPeriod);   // Wait for half the period
  }
}
void printSensorValues(LineSensor& lineSensor, bool isCurrentlyOnLine) {
  Serial.print("Online: ");
  Serial.println(isCurrentlyOnLine);
  Serial.print("Sensor Values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    float sensorValue = lineSensor.readSensor(i);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorValue);
    Serial.print(" ");
  }
  Serial.println();
}
void reportValues() {
  //Serial.println("min/max Values:");
  if(r>0){
    r--;
  }
  else if(r <1){for (int i = 0; i < NUM_SENSORS; i++) {
    
    Serial.print("min v ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(minValues[i]);
    Serial.print("\t max v ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(maxValues[i]);
    Serial.print("\t  v ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensor.readSensor(i));
  }r=500;}
}

void turn180Degrees(RobotOdometry& odometer, MotorControl& motor, LineSensor& lineSensor, int threshold[]) {
  // Get the current orientation
  float currentOrientation = odometer.getOrientationThetaDegrees();

  // Calculate the target orientation (180 degrees from the current orientation)
  float targetOrientation = currentOrientation + 180.0;

  // Ensure that the target orientation wraps around between -180 and 180 degrees
  if (targetOrientation > 180.0) {
    targetOrientation -= 360.0;
  } else if (targetOrientation < -180.0) {
    targetOrientation += 360.0;
  }

  // Set the motor speed to make the robot turn
  motor.moveLeft(30);  // Adjust the turn direction and speed as needed

  // Use a control loop to adjust the robot's orientation
  while (true) {
    // Get the current orientation
    odometer.update(count_e0, count_e1);
    currentOrientation = odometer.getOrientationThetaDegrees();
    Serial.println(odometer.getOrientationThetaDegrees());

    // Check if the robot has reached the target orientation with a small margin of error
    if (fabs(currentOrientation - targetOrientation) < 10.0) {
      // Stop the robot when it's close to the target orientation
      motor.stopMotors();
      break;
    }

    // Check if the robot detects a line
    int sensor0Value = lineSensor.readSensor(0);
    int sensor4Value = lineSensor.readSensor(4);

    if (sensor0Value > threshold[0] || sensor4Value > threshold[4]) {
      // If a line is detected, exit the 180-degree turn and stop
      motor.stopMotors();
      break;
    }
  }
}

/*void reportThresholdValues() {
  Serial.println("Threshold Values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(threshold[i]);
  }
}*/