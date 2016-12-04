const int smsPowerPin = 22;

void smsPower(bool powerState) {
  // Power on GPRS and set proper modes for operation
  if (powerState) {
    digitalWrite(smsPowerPin, HIGH);
    delay(500);
    digitalWrite(smsPowerPin, LOW);

    delay(5000);

    Serial1.println("ATE0");
    delay(100);
    Serial1.println("ATQ1");
    delay(100);
    Serial1.println("ATV0");
    delay(100);
    Serial1.println("AT+CMGF=1");
    delay(100);
    Serial1.println("AT+CNMI=2,2,0,0,0");
    delay(100);
    smsFlush();
  }

  // Power off GPRS
  else {
    digitalWrite(smsPowerPin, HIGH);
    delay(1000);
    digitalWrite(smsPowerPin, LOW);

    while (!Serial1.available()) {
      delay(10);
    }
    smsFlush();
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(19200);

  smsPower(false);
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) {
      Serial1.write(Serial.read());
      delay(10);
    }
  }
  if (Serial1.available()) {
    while (Serial1.available()) {
      Serial.write(Serial1.read());
      delay(10);
    }
  }
}

void smsFlush() {
  if (Serial1.available()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      delay(10);
    }
  }
}
