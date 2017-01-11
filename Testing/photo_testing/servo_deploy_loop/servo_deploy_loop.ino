/*
   Servo driver for in-flight photo shoot

   Test Values:
   - Servo MIN = ~20
   - Servo MAX = ~160

   Positions:
   - Retracted = 20
   - Deployed = 100
*/

#include <Servo.h>

const int controlInput = 2;
const int servoPin = 3;

int servoPosRetract = 20;
int servoPosDeploy = 100; // Need to test to obtain proper value

Servo camServo;

void setup() {
  camServo.attach(servoPin);

  Serial.begin(9600);

  int servoPosition = camServo.read();

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
}

void loop() {
  for (int x =  servoPosRetract; x <= servoPosDeploy; x++) {
    camServo.write(x);
    delay(25);
  }

  delay(5000);

  for (int x = servoPosDeploy; x >= servoPosRetract; x--) {
    camServo.write(x);
    delay(25);
  }

  delay(5000);
}
