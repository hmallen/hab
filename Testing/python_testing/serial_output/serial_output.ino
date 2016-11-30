void setup() {
  Serial.begin(115200);
}

void loop() {
  delay(5000);

  Serial.println("$0");

  if (!Serial.available()) {
    while (!Serial.available()) {
      ;
    }
  }
  if (Serial.available()) {
    String cameraInput;
    while (Serial.available()) {
      char c = Serial.read();
      cameraInput += c;
      delay(5);
    }
    Serial.print("$"); 
    Serial.println(cameraInput);
  }
  Serial.println("$Hello!");

  while (true) {
    ;
  }
}
