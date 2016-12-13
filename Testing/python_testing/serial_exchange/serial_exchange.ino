void setup() {
  Serial.begin(115200);
  //Serial1.begin(115200);

  delay(1000);

  Serial.println("Hello, world!");

  delay(5000);

  Serial.println("$0");
}

void loop() {
  if (Serial.available()) {
    String inputString = "";
    while (Serial.available()) {
      char c = Serial.read();
      inputString += c;
      delay(5);
    }
    Serial.println(inputString);
  }
}
