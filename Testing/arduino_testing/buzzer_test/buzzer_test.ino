const int buzzerRelay = 6;

void setup() {
  pinMode(buzzerRelay, OUTPUT); digitalWrite(buzzerRelay, LOW);
  Serial.begin(115200);
}

void loop() {
  Serial.print("Activating buzzer for 0.5 seconds...");
  digitalWrite(buzzerRelay, HIGH);
  delay(500);
  digitalWrite(buzzerRelay, LOW);
  Serial.println("complete.");

  while (true) {
    ;
  }
}

