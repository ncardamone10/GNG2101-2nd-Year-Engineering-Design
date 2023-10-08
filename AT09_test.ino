/* Chariot */

#include <AFMotor.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

#define TRIGGER_PIN  A0  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A1  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). 
                          //Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
AF_DCMotor leftmotor(2);
AF_DCMotor rightmotor(1);

SoftwareSerial mySerial(10, 9); // RX, TX

boolean prox;
int distance;

void setup() {
  // put your setup code here, to run once:
  leftmotor.setSpeed(200);
  rightmotor.setSpeed(200);
  
  mySerial.begin(9600);
  Serial.begin(9600);
}

void sensor_read() {
  delay(250);
  distance=sonar.ping_cm();
  Serial.println(distance);
  if (distance > 10 or distance == 0) {
    Serial.println ("No obstruction");
    prox=true;
  } else {
    Serial.println("Obstruction");
    prox=false;
  }
}

void loop() {
  char reply[100];
  int i = 0;
  while (mySerial.available()) {
    reply[i] = mySerial.read();
    i += 1;
  }
  sensor_read();
  //end the string
  reply[i] = '\0';
  if(strlen(reply) > 0){
      Serial.println(" pressed");
      Serial.println(reply);
      if (reply[0] == 'C') { // Go backward! Yes, I know that it says run forward
        if (prox == true) {
        rightmotor.run(FORWARD);
        leftmotor.run(FORWARD);
        Serial.println(reply);
          }
        else {
         rightmotor.run(RELEASE);
         leftmotor.run(RELEASE);
          }
        }
      if (reply[0] == 'A') { // Go forward! Yes, I know that it says run backward
        rightmotor.run(BACKWARD);
        leftmotor.run(BACKWARD);
        Serial.println(reply);
        }
      if (reply[0] == 'D') { //turn left
        rightmotor.run(BACKWARD);
        leftmotor.run(FORWARD);
        delay(250);
        rightmotor.run(RELEASE);
        leftmotor.run(RELEASE);
        Serial.println(reply);
        }
      if (reply[0] == 'B') { //turn right
        rightmotor.run(FORWARD);
        leftmotor.run(BACKWARD);
        delay(250);
        rightmotor.run(RELEASE);
        leftmotor.run(RELEASE);
        Serial.println(reply);
        }
    } else {
      Serial.println("released");
      rightmotor.run(RELEASE);
      leftmotor.run(RELEASE);
    }
}
