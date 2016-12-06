#include <Servo.h>

Servo selfieServo;

int pos = 20;
const int posInitial = pos;

const int servoPin = 13;

void setup() {
  Serial.begin(115200);

  selfieServo.attach(servoPin);
}

void loop() {
  for (pos; pos <= (180 - posInitial); pos++) {
    selfieServo.write(pos);
    Serial.print("Pos: "); Serial.println(pos);
    delay(20);
  }

  for (pos; pos >= (180 - posInitial); pos--) {
    selfieServo.write(pos);
    Serial.print("Pos: "); Serial.println(pos);
    delay(20);
  }

  Serial.println("Moving opposite direction to maximum in 5 seconds.");
  delay(5000);

  for (pos; pos <= (180 - posInitial); pos++) {
    selfieServo.write(pos);
    Serial.print("Pos: "); Serial.println(pos);
    delay(20);
  }

  Serial.println("Moving opposite direction to maximum in 5 seconds.");
  delay(5000);

  for (pos; pos >= posInitial; pos--) {
    selfieServo.write(pos);
    Serial.print("Pos: "); Serial.println(pos);
    delay(20);
  }
  delay(2500);

  Serial.print("Detaching servo...");
  selfieServo.detach();
  Serial.println("complete.");
  delay(2500);
  Serial.print("Attaching servo...");
  selfieServo.attach(servoPin);
  Serial.println("complete.");
  delay(2500);

  Serial.print("Moving servo to opposite maximum...");
  for (pos; pos <= (180 - posInitial); pos++) {
    selfieServo.write(pos);
    Serial.print("Pos: "); Serial.println(pos);
    delay(20);
  }
  Serial.println("complete.");

  while (true) {
    ;
  }
}
