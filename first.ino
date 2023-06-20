#include "SRF05.h"

// Pin connections
const int triggerPin = 2;    // Trigger pin of SRF05
const int echoPin = 3;       // Echo pin of SRF05
const int pumpPin = 9;       // Control pin for water pump


// PID constants
const double Kp = 2.0;       // Proportional gain
const double Ki = 0.5;       // Integral gain
const double Kd = 0.1;       // Derivative gain

// Setpoint and variables
const double setpoint = 7;  // Desired water level in centimeters
double currentLevel = 0;     // Current water level in centimeters
double error = 0;            // Error term
double previousError = 0;    // Previous error term
double integral = 0;         // Integral term
double derivative = 0;       // Derivative term
double output = 0;           // Output signal for the pump

// Timing variables
unsigned long previousTime = 0;
const int sampleTime = 100;  // Sample time in milliseconds

// Create an instance of the SRF05 class
SRF05 SRF05(triggerPin, echoPin);

void setup() {
  // Initialize the pump pin as an output
  pinMode(pumpPin, OUTPUT);
  Serial.begin(9600);

  // Initialize the SRF05 sensor
  
}

void loop() {

  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);
  // Read the current water level from the SRF05 sensor
  currentLevel = 14 - SRF05.getCentimeter();
  

  // Calculate the error term
  error = setpoint - currentLevel;

  // Calculate the time difference
  unsigned long currentTime = millis();
  double timeDiff = (currentTime - previousTime) / 1000.0; // Convert to seconds

  // PID calculations
  integral += error * timeDiff;
  derivative = (error - previousError) / timeDiff;
  output = Kp * error + Ki * integral + Kd * derivative;

  // Limit the output signal to the allowed range
  output = constrain(output, 0, 255);

  // Update the previous error and time variables
  previousError = error;
  previousTime = currentTime;

  // Control the water pump based on the output signal
  analogWrite(pumpPin, output);

  Serial.print("Water Level: ");
  Serial.print(currentLevel);
  Serial.print("  Pump Speed: ");
  Serial.print(output);
  Serial.print("  I: ");
  Serial.print(integral);
  Serial.print("  D: ");
  Serial.println(derivative);

  // Delay for the sample time
  delay(sampleTime);
}
