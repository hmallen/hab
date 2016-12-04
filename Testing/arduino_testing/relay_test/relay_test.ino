void setup() {
  pinMode(4, OUTPUT); digitalWrite(4, LOW);
  pinMode(5, OUTPUT); digitalWrite(5, LOW);
  pinMode(6, OUTPUT); digitalWrite(6, LOW);
  Serial.begin(115200);
}

void loop() {
  Serial.println("RELAY #4 ACTIVATED");
  digitalWrite(4, HIGH);
  delay(5000);
  Serial.println("RELAY #3 ACTIVATED");
  digitalWrite(4, LOW); digitalWrite(5, HIGH);
  delay(5000);
  Serial.println("RELAY #2 ACTIVATED");
  digitalWrite(5, LOW); digitalWrite(6, HIGH);
  delay(5000);
  digitalWrite(6, LOW);
}

