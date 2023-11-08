#ifndef LineSensor_h
#define LineSensor_h

// Define constants for pin numbers
#define EMIT 11  // Pin for emitting light
#define DN1 12   // Digital sensor pin 1
#define DN2 18   // Digital sensor pin 2
#define DN3 20   // Digital sensor pin 3
#define DN4 21   // Digital sensor pin 4
#define DN5 22   // Digital sensor pin 5
#define NUM_SENSORS 5  // The number of sensors

#include "Arduino.h"

class LineSensor {
public:
  // Constructor to initialize the LineSensor class with parameters
  LineSensor(int emitPin, int* sensorPins, int numSensors);

  // Method to begin using the line sensors
  void begin();

  // Method to read the value from a specific sensor
  float readSensor(int sensorNumber);

  // Method to read values from all sensors and store them in an array
  void readAllSensors(float* values);

  // Method to normalize the sensor reading to a specific range
  float normalizeSensorRange(int sensorNumber, int minValue, int maxValue);

  // Method to normalize and map the sensor value to a specific range
  float normalizeAndMapSensorValue(int sensorNumber, int minValue, int maxValue);

  // Method to set calibration values for sensors
  void setCalibrationValues(int* maxVals, int* minVals);

private:
  int _emitPin;       // Pin for emitting light
  int* _sensorPins;   // Array of digital sensor pins
  int _numSensors;    // Number of sensors
  int* maxValues;     // Array to store maximum calibration values for sensors
  int* minValues;     // Array to store minimum calibration values for sensors
};

#endif
