/*
  HAB Heartbeat

  Features:
  - Listens for "heartbeat" signal between sensor reads by Arduino Mega
  - Stores current program state in EEPROM
  - Triggers reset and restoration of program state if Arduino Mega stalls

  Considerations:
  - Count quick, repetitive pulses to read state of Arduino Mega
*/

#define READYPINHOLDTIME 10000 // Time (ms) that program ready pin must remain HIGH to proceed

const int programReadyPin = 2;
const int heartbeatInputPin = 3;
const int resetPin = 4;

void setup() {
  pinMode(resetPin, OUTPUT); digitalWrite(resetPin, HIGH);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);
  pinMode(programReadyPin, INPUT_PULLUP);
  pinMode(heartbeatInputPin, INPUT_PULLUP);

  Serial.begin(9600);

  delay(1000);

  triggerReset(true);
}

void loop() {
  unsigned long startTime = millis();

  while ((millis() - startTime) < 10000) {
    if (digitalRead(heartbeatInputPin) == HIGH) {
      startTime = millis();
      for (int x = 0; x < 2; x++) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
      }
    }
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);

  triggerReset(false);
}

void triggerReset(bool waitOnly) {
  if (!waitOnly) {
    Serial.println();
    Serial.print("Initiating reset...");
    digitalWrite(resetPin, LOW);
    delay(1000);
    digitalWrite(resetPin, HIGH);
    delay(1000);
    Serial.println("complete.");
  }

  Serial.print("Waiting for program start signal...");
  unsigned long startTime = millis();
  while ((millis() - startTime) < READYPINHOLDTIME) {
    if (digitalRead(programReadyPin) == LOW) startTime = millis();
    delay(10);
  }
  Serial.println("received.");
  digitalWrite(LED_BUILTIN, HIGH);
}
