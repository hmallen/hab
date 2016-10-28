/*
  HAB Heartbeat

  Features:
  - Listens for "heartbeat" signal between sensor reads by Arduino Mega
  - Stores current program state in EEPROM
  - Triggers reset and restoration of program state if Arduino Mega stalls

  Considerations:
  - Count quick, repetitive pulses to read state of Arduino Mega
*/

void setup() {
  pinMode(XXX, OUTPUT); digitalWrite(XXX, LOW);
  pinMode(YYY, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  int inputPulses = 0;
  if (digitalRead(YYY, HIGH) {
  inputPulses++
  while (digitalRead(YYY, HIGH) {
    ;
  }
  for (unsigned long startTime = millis(); (millis() - startTime) < 1000; ) {
    ;
  }
  int inputPulses = 1;

}
}
