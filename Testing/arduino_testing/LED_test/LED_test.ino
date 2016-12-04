const int gpsReadyLED = 28; // Multi-color LED round-side input [Green]
const int programStartLED = 29; // Multi-color LED flat-side input [Red]

void setup() {
  pinMode(gpsReadyLED, OUTPUT); digitalWrite(gpsReadyLED, LOW);
  pinMode(programStartLED, OUTPUT); digitalWrite(programStartLED, LOW);
  Serial.begin(115200);

  digitalWrite(gpsReadyLED, HIGH);
}

void loop() {
  Serial.println("GPS ready LED ON.");
  delay(2500);
  Serial.println("Program start LED ON.");
  digitalWrite(gpsReadyLED, LOW);
  digitalWrite(programStartLED, HIGH);
  delay(2500);
  digitalWrite(programStartLED, LOW);
  digitalWrite(gpsReadyLED, HIGH);
}
