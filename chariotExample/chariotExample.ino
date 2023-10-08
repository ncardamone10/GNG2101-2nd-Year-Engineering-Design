// Chariot - A simple Arduino robot control code 
// WARNING: This code has been refactored by chatgpt and is untested. Use at your own risk

// Include necessary libraries for motor control, ultrasonic sensor, and software serial communication
#include "arduino libraries_2101\adafruit-Adafruit-Motor-Shield-library-89a0973\AFMotor.h"
#include "arduino libraries_2101\NewPing\NewPing.h"
#include "arduino libraries_2101\SoftwareSerial\src\SoftwareSerial.h"


// Define symbolic constants for ultrasonic sensor pins and maximum distance
#define TRIGGER_PIN  A0                                 // Define the pin number for the trigger pin of the ultrasonic sensor
#define ECHO_PIN     A1                                 // Define the pin number for the echo pin of the ultrasonic sensor
#define MAX_DISTANCE 200                                // Define the maximum distance (in cm) up to which the sensor should detect objects

// Initialize objects for ultrasonic sensor and motors
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);     // Create a NewPing object for controlling the ultrasonic sensor
AF_DCMotor leftmotor(2);                                // Create a motor object to control the left motor
AF_DCMotor rightmotor(1);                               // Create a motor object to control the right motor
SoftwareSerial mySerial(10, 9);                         // Create a SoftwareSerial object for communication through pins 10 (RX) and 9 (TX)

// Declare global variables
boolean prox;                                           // Boolean variable to store whether the path is clear (true) or obstructed (false)
int distance;                                           // Integer variable to store the distance measured by the ultrasonic sensor

void setup() {
  // Initialize motors and serial communication
  initializeMotors();                                   // Call the function to initialize motors
  initializeSerialCommunication();                      // Call the function to initialize serial communication
}

// Main loop where the program runs over and over
void loop() {
  char reply[100];                                      // Array to store received serial data
  int i = 0;                                            // Index variable for array
  
  // Read incoming serial data and store it in the 'reply' array
  while (mySerial.available()) {
    reply[i] = mySerial.read();                         // Store each byte of data in the array
    i++;                                                // Move to the next array index
  }
  
  sensor_read();                                        // Call function to check for obstructions
  reply[i] = '\0';                                      // End the string with null termination 
  
  // Check if any serial data was received and control motors accordingly
  if(strlen(reply) > 0){
      Serial.println("Button pressed");                 // Print message indicating a button was pressed
      Serial.println(reply);                            // Print the received data
      moveMotors(reply[0]);                             // Call function to control motors based on the first character of received data
  } else {
    Serial.println("Button released");                  // Print message indicating the button was released
    stopMotors();                                       // Call function to stop motors
  }
}

// Function to initialize motors by setting their speed
void initializeMotors() {
  leftmotor.setSpeed(200);                              // Set the speed of the left motor to 200
  rightmotor.setSpeed(200);                             // Set the speed of the right motor to 200
}

// Function to initialize serial communication
void initializeSerialCommunication() {
  mySerial.begin(9600);                                 // Start software serial communication at 9600 baud rate
  Serial.begin(9600);                                   // Start serial monitor communication at 9600 baud rate
}

// Function to read data from ultrasonic sensor and check for obstructions
void sensor_read() {
  delay(250);                                           // Wait for 250 milliseconds to ensure accurate readings
  distance = sonar.ping_cm();                           // Measure the distance using the ultrasonic sensor and store it in the 'distance' variable
  Serial.println(distance);                             // Print the measured distance to the serial monitor
  
  // Check if the path is clear or obstructed and update the 'prox' variable accordingly
  if (distance > 10 || distance == 0) {
    Serial.println("No obstruction");                   // Print message indicating no obstruction
    prox = true;                                        // Set 'prox' to true indicating path is clear
  } else {
    Serial.println("Obstruction");                      // Print message indicating obstruction
    prox = false;                                       // Set 'prox' to false indicating path is obstructed
  }
}

// Function to control motors based on received command
void moveMotors(char command) {
  // Check the received command and control motors accordingly
  // 'C' - Forward, 'A' - Backward, 'D' - Turn Left, 'B' - Turn Right
  if (command == 'C' && prox == true) {                 // Move forward if path is clear
    runMotors(FORWARD, FORWARD);
  } else if (command == 'A') {                          // Move backward
    runMotors(BACKWARD, BACKWARD);
  } else if (command == 'D') {                          // Turn left
    runMotors(BACKWARD, FORWARD);
    delay(250);                                         // Pause for 250 milliseconds while turning
    stopMotors();                                       // Stop motors after turning
  } else if (command == 'B') {                          // Turn right
    runMotors(FORWARD, BACKWARD);
    delay(250);                                         // Pause for 250 milliseconds while turning
    stopMotors();                                       // Stop motors after turning
  }
}

// Function to run motors in specified directions
void runMotors(int leftMotorDirection, int rightMotorDirection) {
  leftmotor.run(leftMotorDirection);                    // Run left motor in specified direction
  rightmotor.run(rightMotorDirection);                  // Run right motor in specified direction
}

// Function to stop both motors
void stopMotors() {
  leftmotor.run(RELEASE);                               // Stop the left motor
  rightmotor.run(RELEASE);                              // Stop the right motor
}
