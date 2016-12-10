#include <MS5xxx.h>
#include <Wire.h>

float ms5607Temp, ms5607Press;

MS5xxx ms5607(&Wire);

void setup() {
  Serial.begin(115200);

  if (sensor.connect() > 0) {
    Serial.println("Error connecting...");
    delay(500);
    setup();
  }
}

void loop() {
  for (int x = 0; x < 3; x++) {
    ms5607.ReadProm();
    ms5607.Readout();

    ms5607Temp = ms5607.GetTemp() / 100.0;
    ms5607Press = ms5607.GetPres() / 100.0;

    if ((-100.0 <= ms5607Temp <= 100.0) && (0.0 <= ms5607Press <= 1100.0)) return true;

    delay(100);
  }

  Serial.print(ms5607Temp);
  Serial.print(" / ");
  Serial.println(ms5607Press);

  delay(2500);
}
