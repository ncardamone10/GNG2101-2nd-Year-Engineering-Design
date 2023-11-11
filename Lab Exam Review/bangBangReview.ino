// Define the pin for the temperature sensor and the heater (LED)
const int ___ = A0;  // Fill in the variable name for the temperature sensor pin
const int ___ = 13;  // Fill in the variable name for the heater pin

// Define the target temperature
const float ___ = 22.0; // Fill in the variable name for the target temperature

void setup() {
  // Initialize the heater pin as an output
  pinMode(___, ____); // Fill in the correct variable name for the heater pin

  // Start the serial communication at 9600 baud
  Serial.begin(___); 
}

void loop() {
  // Read the value from the temperature sensor
  int sensorValue = analogRead(___); // Fill in the correct variable name for the sensor pin

  // Convert the sensor reading to temperature in Celsius
  float voltage = sensorValue * 5.0 / 1023.0; 
  float temperature = (voltage - 0.5) * 100.0; 

  // Display the temperature in the Serial Monitor
  Serial.print("Current Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  // Call the ___ function to control the heater
  ___(temperature); // Fill in the correct function name and its argument

  // Wait for a bit before reading the temperature again
  delay(2000); // This can remain as is
}

void bangBang(float temperature) { // Fill in the function name and its parameter name
  // Bang-Bang control logic
  if (________) { // Fill in the correct variable names
    // If the temperature is below the target, turn the heater on
    digitalWrite(___, HIGH); // Fill in the correct variable name for the heater pin
    Serial.println("Heater ON");
  } else {
    // If the temperature is at or above the target, turn the heater off
    digitalWrite(___, LOW); // Fill in the correct variable name for the heater pin
    Serial.println("Heater OFF");
  }
}
