void setup() {
  pinMode(2, OUTPUT); digitalWrite(2, LOW);
  pinMode(3, OUTPUT); digitalWrite(3, LOW);

  Serial.begin(115200);
}

void loop() {
  Serial.println("Pin #2 HIGH");
  digitalWrite(2, HIGH);
  delay(5000);
  Serial.println("Pin #3 HIGH");
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  delay(5000);
  digitalWrite(3, LOW);
}

