#include <PID_v1.h>

// Define the connections
const int ___ = 3;  // PWM output to motor driver, fill in the variable name
const int ___ = A0; // Input from speed sensor, fill in the variable name

// Define variables for PID
double ___, ___, ___; // Fill in the variable names for setSpeed, currentSpeed, and motorOutput

// Specify the links and initial tuning parameters
double Kp = 1.0, Ki = 2.3, Kd = 0.1; // Fill in the PID constants (P, I, D)
PID myPID(&currentSpeed, &motorOutput, &setSpeed, Kp, Ki, Kd, DIRECT); 

void setup() {
  // Initialize the motor speed as an output
  pinMode(___, OUTPUT); // Fill in the correct variable name for motor speed output

  // Initialize the PID controller
  ___.SetMode(AUTOMATIC); // Fill in the PID object name

  // Define the set speed
  ___ = 100; // Fill in the variable name for set speed and its value
}

void loop() {
  // Read the current speed from sensor
  ___ = analogRead(___); // Fill in the variable names for current speed and speed sensor

  // Compute PID
  ___.Compute(); // Fill in the PID object name

  // Adjust the motor speed
  analogWrite(___, ___); // Fill in the motor speed output pin and motor output variable

  // Add a delay for stability
  delay(100);
}

// PID Loop Explanation:
// - The PID controller continuously calculates an error value as the difference between a desired setpoint (setSpeed) and a measured process variable (currentSpeed).
// - It applies a correction (motorOutput) based on proportional (Kp), integral (Ki), and derivative (Kd) terms.
// - This correction adjusts the motor speed to match the set speed.
