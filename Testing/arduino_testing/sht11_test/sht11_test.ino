#include <SHT1x.h>

const int shtData = 2;
const int shtClock = 3;

SHT1x sht(shtData, shtClock);

void setup() {
  Serial.begin(115200);
}

void loop() {
  float shtTemp = sht.readTemperatureC();
  float shtHumidity = sht.readHumidity();
  Serial.print(shtTemp); Serial.print(" / "); Serial.println(shtHumidity);
  delay(2500);
}

