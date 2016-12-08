#include <DHT.h>

const int dhtPin = 2;

DHT dht(dhtPin, DHT11);

void setup() {
  Serial.begin(115200);

  dht.begin();
}

void loop() {
  int dhtTemp = dht.readTemperature();
  delay(500);
  int dhtHumidity = dht.readHumidity();
  Serial.print("Temperature(C) = ");
  Serial.println(dhtTemp);
  Serial.print("Humidity(%RH) =  ");
  Serial.println(dhtHumidity);
  Serial.println();
  delay(2000);
}
