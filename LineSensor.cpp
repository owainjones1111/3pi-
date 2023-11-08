#include "LineSensor.h"

// Constructor to initialize the LineSensor class with parameters
LineSensor::LineSensor(int emitPin, int* sensorPins, int numSensors) {
  _emitPin = emitPin;
  _sensorPins = sensorPins;
  _numSensors = numSensors;
}

// Method to set up the line sensors
void LineSensor::begin() {
  pinMode(_emitPin, INPUT);  // Set EMIT as an input (off)
  for (int i = 0; i < _numSensors; i++) {
    pinMode(_sensorPins[i], INPUT);  // Set line sensor pins to input
  }
}

// Method to set calibration values for the sensors
void LineSensor::setCalibrationValues(int* maxVals, int* minVals) {
  maxValues = maxVals;
  minValues = minVals;
}

// Method to read the value from a specific sensor
float LineSensor::readSensor(int sensorNumber) {
  if (sensorNumber < 0 || sensorNumber >= _numSensors) {
    return 0.0;  // Return 0.0 if the sensor number is out of range
  }

  pinMode(_emitPin, OUTPUT);
  digitalWrite(_emitPin, HIGH);
  pinMode(_sensorPins[sensorNumber], OUTPUT);
  digitalWrite(_sensorPins[sensorNumber], HIGH);
  delayMicroseconds(10);
  pinMode(_sensorPins[sensorNumber], INPUT);

  unsigned long start_time = micros();
  while (digitalRead(_sensorPins[sensorNumber]) == HIGH) {
  }
  unsigned long end_time = micros();

  pinMode(_sensorPins[sensorNumber], INPUT);
  unsigned long elapsed_time = end_time - start_time;
  return static_cast<float>(elapsed_time);
}

// Method to read values from all sensors and store them in an array
void LineSensor::readAllSensors(float* values) {
  for (int i = 0; i < _numSensors; i++) {
    values[i] = readSensor(i);
  }
}

// Method to normalize the sensor reading to a specific range
float LineSensor::normalizeSensorRange(int sensorNumber, int minValue, int maxValue) {
  if (sensorNumber < 0 || sensorNumber >= _numSensors) {
    return 0.0;  // Return 0.0 if the sensor number is out of range
  }

  // Calculate the original sensor value range
  float originalRange = maxValues[sensorNumber] - minValues[sensorNumber];

  // Check if the original range is zero to avoid division by zero
  if (originalRange == 0.0) {
    return 0.0;
  }

  // Read the sensor value
  float sensorValue = readSensor(sensorNumber);

  // Calculate the normalized value within the specified range
  float normalizedValue = (sensorValue - minValues[sensorNumber]) / originalRange;

  // Scale the normalized value to the desired range
  float scaledValue = minValue + normalizedValue * (maxValue - minValue);

  // Ensure the scaled value is within the specified range
  if (scaledValue < minValue) {
    return minValue;
  } else if (scaledValue > maxValue) {
    return maxValue;
  } else {
    return scaledValue;
  }
}

// Method to normalize and map the sensor value to a specific range
float LineSensor::normalizeAndMapSensorValue(int sensorNumber, int minValue, int maxValue) {
  // Normalize the sensor value and map it to the specified range
  return normalizeSensorRange(sensorNumber, minValue, maxValue);
}
