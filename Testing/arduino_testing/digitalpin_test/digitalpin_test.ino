const int testPin = 22;

void setup() {
  pinMode(testPin, OUTPUT); digitalWrite(testPin, LOW);

  Serial.begin(115200);
}

void loop() {
  Serial.println("HIGH");
  digitalWrite(testPin, HIGH);
  delay(2500);
  Serial.println("LOW");
  digitalWrite(testPin, LOW);
  delay(2500);
}
