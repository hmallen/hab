const int lightPin = A0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int lightVal = analogRead(lightPin);
  Serial.println(lightVal);
  delay(1000);
}

