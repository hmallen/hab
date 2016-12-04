/*
  HAB Heartbeat

  Features:
  - Listens for "heartbeat" signal between sensor reads by Arduino Mega
  - Stores current program state in EEPROM
  - Triggers reset and restoration of program state if Arduino Mega stalls

  Considerations:
  - Count quick, repetitive pulses to read state of Arduino Mega
  - HEARTBEAT LED OUTPUT CAUSES VOLTAGE DROP IN GPRS!!!!
*/

#include <avr/wdt.h>

#define READYPINHOLDTIME 5000 // Time (ms) that program ready pin must remain HIGH to proceed

const int programReadyPin = 2;
const int heartbeatInputPin = 3;
const int resetPin = 4;
//const int heartbeatLED = 13;

void watchdogSetup() {
  cli();
  wdt_reset();
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (0 << WDIE) | (1 << WDE) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (1 << WDP0);
  sei();
}

void setup() {
  watchdogSetup();

  pinMode(resetPin, OUTPUT); digitalWrite(resetPin, HIGH);
  //pinMode(heartbeatLED, OUTPUT); digitalWrite(heartbeatLED, LOW);
  pinMode(programReadyPin, INPUT_PULLUP);
  pinMode(heartbeatInputPin, INPUT_PULLUP);

  //Serial.begin(9600);

  delay(1000);

  triggerReset(true);
}

void loop() {
  unsigned long startTime = millis();

  while ((millis() - startTime) < 10000) {
    if (digitalRead(programReadyPin) == LOW) {
      //digitalWrite(heartbeatLED, LOW);
      while (digitalRead(programReadyPin) == LOW) {
        wdt_reset();
        delay(10);
      }
      //digitalWrite(heartbeatLED, HIGH);
      startTime = millis();
    }
    if (digitalRead(heartbeatInputPin) == HIGH) {
      startTime = millis();
      /*for (int x = 0; x < 2; x++) {
        digitalWrite(heartbeatLED, LOW);
        delay(100);
        digitalWrite(heartbeatLED, HIGH);
        delay(100);
        }*/
    }
    wdt_reset();
    delay(10);
  }
  triggerReset(false);
}

void triggerReset(bool waitOnly) {
  if (!waitOnly) {
    //Serial.println();
    //Serial.print("Initiating reset...");
    digitalWrite(resetPin, LOW);
    delay(1000);
    digitalWrite(resetPin, HIGH);
    delay(1000);
    //Serial.println("complete.");
  }

  //Serial.print("Waiting for program start signal...");
  unsigned long startTime = millis();
  while ((millis() - startTime) < READYPINHOLDTIME) {
    wdt_reset();
    if (digitalRead(programReadyPin) == LOW) startTime = millis();
  }
  //Serial.println("received.");
  //digitalWrite(heartbeatLED, HIGH);
}
