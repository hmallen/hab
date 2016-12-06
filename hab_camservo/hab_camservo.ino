/*
   Servo driver for in-flight photo shoot
*/

#include <Servo.h>

const bool debugMode = true;

const int controlInput = 8;
const int servoPin = 9;

int servoPositionOut = 20;
int servoPositionIn = 110; // Need to test to obtain proper value

Servo camServo;

void setup() {
  pinMode(controlInput, INPUT_PULLUP);
  camServo.attach(servoPin);

  Serial.begin(115200);

  if (debugMode) Serial.print("Moving servo to start position...");
  int servoPositionInitial = camServo.read();
  if (servoPositionInitial > servoPositionOut) {
    for (int x = servoPositionInitial; x >= servoPositionOut; x--) {
      camServo.write(x);
      delay(15);
    }
  }
  else if (servoPositionInitial < servoPositionOut) {
    for (int x = servoPositionInitial; x <= servoPositionOut; x++) {
      camServo.write(x);
      delay(15);
    }
  }
  if (debugMode) Serial.println("complete.");
}

void loop() {
  delay(100);
  if (digitalRead(controlInput) == LOW) {
    if (debugMode) Serial.print("Positioning photo...");
    positionPhoto();
  }
}

void positionPhoto() {
  for (int x =  servoPositionOut; x <= servoPositionIn; x++) {
    camServo.write(x);
    delay(15);
  }
  while (digitalRead(controlInput) == LOW) {
    ;
  }
  for (int x = servoPositionIn; x >= servoPositionOut; x--) {
    camServo.write(x);
    delay(15);
  }
  if (debugMode) Serial.println("capture complete.");
}
