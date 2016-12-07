const int gasRelay = 6;
const int gasOne = A8;
const int gasSix = A13;

void setup() {
  //pinMode(gasRelay, OUTPUT); digitalWrite(gasRelay, LOW);
  pinMode(gasRelay, OUTPUT); digitalWrite(gasRelay, HIGH);

  Serial.begin(115200);
}

void loop() {
  //Serial.println("ON");
  //digitalWrite(gasRelay, HIGH);
  //delay(5000);
  int gasOneVal = analogRead(gasOne);
  int gasSixVal = analogRead(gasSix);
  Serial.print("#1: ");
  Serial.println(gasOneVal);
  Serial.print("#6: ");
  Serial.println(gasSixVal);
  Serial.println();
  //Serial.println("OFF");
  //digitalWrite(gasRelay, LOW);
  //delay(5000);
  delay(1000);
}
