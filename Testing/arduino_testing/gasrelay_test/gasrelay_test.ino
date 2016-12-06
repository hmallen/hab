const int gasRelay = 4;

void setup() {
  pinMode(gasRelay, OUTPUT); digitalWrite(gasRelay, LOW);

  Serial.begin(115200);
}

void loop() {
  Serial.println("ON");
  digitalWrite(gasRelay, HIGH);
  delay(5000);
  Serial.println("OFF");
  digitalWrite(gasRelay, LOW);
  delay(5000);
}
