/*
   Servo driver for in-flight photo shoot

   Test Values:
   - Servo MIN = ~20
   - Servo MAX = ~160
*/

#include <Servo.h>

const bool debugMode = true;
const bool debugPosTestMode = true;

const int controlInput = 2;
const int servoPin = 3;

int servoPosRetract = 20;
int servoPosDeploy = 100; // Need to test to obtain proper value

Servo camServo;

void setup() {
  pinMode(controlInput, INPUT_PULLUP);
  camServo.attach(servoPin);

  Serial.begin(9600);

  int servoPosition = camServo.read();
  if (debugMode) {
    Serial.print("Servo position: ");
    Serial.println(servoPosition);
    Serial.print("Moving servo to start position...");
  }

  if (servoPosition > servoPosRetract) {
    for (int x = servoPosition; x >= servoPosRetract; x--) {
      camServo.write(x);
      delay(25);
    }
  }
  else if (servoPosition < servoPosRetract) {
    for (int x = servoPosition; x <= servoPosRetract; x++) {
      camServo.write(x);
      delay(25);
    }
  }
  servoPosition = camServo.read();
  if (debugMode) {
    Serial.println("complete.");
    Serial.print("Servo position: ");
    Serial.println(servoPosition);
  }

  if (debugPosTestMode) {
    delay(5000);

    Serial.println("Servo --> 90");
    for (int x = 20; x <= 90; x++) {
      camServo.write(x);
      delay(25);
    }

    delay(5000);

    Serial.println("Servo --> 100");
    for (int x = 90; x <= 100; x++) {
      camServo.write(x);
      delay(25);
    }

    delay(5000);

    Serial.println("Servo --> 110");
    for (int x = 100; x <= 110; x++) {
      camServo.write(x);
      delay(25);
    }

    delay(5000);

    Serial.println("Servo --> 120");
    for (int x = 110; x <= 120; x++) {
      camServo.write(x);
      delay(25);
    }

    delay(5000);

    Serial.println("Moving to start position. Servo --> 20");
    for (int x = 120; x >= 20; x--) {
      camServo.write(x);
      delay(25);
    }

    Serial.println("Testing complete.");

    while (true) {
      ;
    }
  }
}

void loop() {
  delay(100);
  if (digitalRead(controlInput) == LOW) {
    if (debugMode) Serial.print("Positioning photo...");
    positionPhoto();
  }
}

void positionPhoto() {
  for (int x =  servoPosRetract; x <= servoPosDeploy; x++) {
    camServo.write(x);
    delay(25);
  }
  while (digitalRead(controlInput) == LOW) {
    ;
  }
  for (int x = servoPosDeploy; x >= servoPosRetract; x--) {
    camServo.write(x);
    delay(25);
  }
  if (debugMode) Serial.println("capture complete.");
}
