
const int pumpPin = 9;              // PWM pin for controlling pump speed
const int pin1 = 6;                 //direction pins
const int pin2 = 7;                 //direction pins
const int triggerPin = 2;           // Ultrasonic sensor trigger pin
const int echoPin = 3;              // Ultrasonic sensor echo pin

// PID Constants
const float Kp = 0.5;               // Proportional gain
const float Ki = 0.2;               // Integral gain
const float Kd = 0.1;               // Derivative gain

// Variables
const int targetLevel = 8;        // Desired water level (adjust as needed)
int pumpSpeed = 0;                  // Initial pump speed (0-255)
float waterLevel = 0;               // Current water level
int previousError = 0;              // Previous error for derivative term
float integralTerm = 0;             // Integral term for integral term
unsigned long previousTime = 0;     // Previous time for derivative term
#include "SRF05.h"
SRF05 SRF(triggerPin,echoPin);
void setup() {
  // Initialize pump and ultrasonic sensor pins
  pinMode(pumpPin, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Start serial communication for debugging (optional)
  Serial.begin(9600);
  SRF.setCorrectionFactor(1.035);
  SRF.setModeAverage(10);
}

void loop() {
   digitalWrite(pin1,HIGH);
   digitalWrite(pin2,LOW);
  // Read water level using ultrasonic sensor
  waterLevel = 14-SRF.getCentimeter();

  // Calculate the error
  int error = targetLevel - waterLevel;

  // Calculate the time difference
  unsigned long currentTime = millis();
  float timeDiff = (currentTime - previousTime) / 1000.0; // Convert to seconds

  // Proportional term
  float proportionalTerm = Kp * error;

  // Integral term
  integralTerm += Ki * error * timeDiff;

  // Derivative term
  float derivativeTerm = Kd * (error - previousError) / timeDiff;

  // Calculate the pump speed using PID control
  pumpSpeed = proportionalTerm + integralTerm + derivativeTerm;

  // Limit the pump speed to prevent overflow
  pumpSpeed = constrain(pumpSpeed, 0, 255);

  // Control the pump speed
  analogWrite(pumpPin, pumpSpeed);

  // Print the water level, pump speed, and PID terms for debugging (optional)
  Serial.print("Water Level: ");
  Serial.print(waterLevel);
  Serial.print("  Pump Speed: ");
  Serial.print(pumpSpeed);
  Serial.print("  P: ");
  Serial.print(proportionalTerm);
  Serial.print("  I: ");
  Serial.print(integralTerm);
  Serial.print("  D: ");
  Serial.println(derivativeTerm);
  Serial.print("  ERROR: ");
  Serial.println(error);

  // Update previous error and time for the next iteration
  previousError = error;
  previousTime = currentTime;

  // Check if the target level is reached
  if (waterLevel >= targetLevel) {
    // Stop the pump
    analogWrite(pumpPin, 0);
    integralTerm=0;
  }

  // Delay between measurements
  delay(1000);
}
