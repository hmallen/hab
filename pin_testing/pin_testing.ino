const int gasRelay = 7;  // Gas sensors
const int heaterRelay = 6;  // Internal payload heater
const int relayPin3 = 5;
const int relayPin4 = 4;
const int smsPowerPin = 22;
const int gpsPpsPin = 23;
const int gpsReadyLED = 24;
const int programStartPin = 25;
const int programStartLED = 26;
const int programReadyPin = 27;
const int heartbeatOutputPin = 28;
const int buzzerPin = 29;

void setup() {
  pinMode(gasRelay, OUTPUT); digitalWrite(gasRelay, HIGH);  // Gas sensors
  pinMode(heaterRelay, OUTPUT); digitalWrite(heaterRelay, LOW); // Payload heater
  pinMode(relayPin3, OUTPUT); digitalWrite(relayPin3, LOW);
  pinMode(relayPin4, OUTPUT); digitalWrite(relayPin4, LOW);
  pinMode(smsPowerPin, OUTPUT); digitalWrite(smsPowerPin, LOW);
  pinMode(gpsReadyLED, OUTPUT); digitalWrite(gpsReadyLED, LOW);
  pinMode(programStartLED, OUTPUT); digitalWrite(programStartLED, LOW);
  pinMode(programReadyPin, OUTPUT); digitalWrite(programReadyPin, LOW);
  pinMode(heartbeatOutputPin, OUTPUT); digitalWrite(heartbeatOutputPin, LOW);
  pinMode(buzzerPin, OUTPUT); digitalWrite(buzzerPin, LOW);
  pinMode(gpsPpsPin, INPUT_PULLUP);
  pinMode(programStartPin, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  digitalWrite(gpsReadyLED, LOW); digitalWrite(programStartLED, LOW);
  for (unsigned long startTime = millis(); (millis() - startTime) < 5000; ) {
    bool startPinVal = digitalRead(programStartPin);
    Serial.print("Start Pin: "); Serial.println(startPinVal);
    delay(500);
  }
  Serial.println("GPS LED HIGH / START LED LOW");
  digitalWrite(gpsReadyLED, HIGH);
  delay(5000);
  Serial.println("GPS LED LOW / START LED HIGH");
  digitalWrite(gpsReadyLED, LOW); digitalWrite(programStartLED, HIGH);
  delay(5000);
}
