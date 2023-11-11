// Define LED pin constants
#define ledPin1 = ___;  // LED connected to digital pin 12 (fill in the blank)
const int ledPin2 = ___;  // LED connected to digital pin 8 (fill in the blank)

// Bonus: explain which is better above, const int or #define

void setup() {
  // Set the LED pins as outputs
  pinMode(ledPin1, ___);  // Fill in the mode 
  pinMode(ledPin2, ___);  // Fill in the mode 

  // Initialize serial communication at 9600 baud (fill in the blank)
  ______.begin(___); 

  // Turn LED 2 on
  _______(ledPin2, _______)____
}

void loop() {
  // Turn on LED 1
  digitalWrite(ledPin1, ___); // Fill in the state 

  // Print a message to the Serial Monitor to say led is on
  Serial.println(___); // Fill in a message

  // Wait for ___ ms (fill in the blank for 1 Hz blink)
  delay(___); 

  // Turn off LED 1
  digitalWrite(_____, ___); // Fill in the state 

  // Print a message to the Serial Monitor to say led is off
  Serial._____(___); // Fill in a message
  
  
}
