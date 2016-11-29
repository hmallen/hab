#include <string.h>

float gpsLat = 30.123456;
float gpsLng = 31.123456;
float gpsAlt = 123.12;
float gpsSpeed = 10.12;
float gpsCourse = 234.56;

void setup() {
  Serial.begin(9600);
}

void loop() {
  char txString[80];
  char gpsLatChar[10];
  char gpsLngChar[10];
  char gpsAltChar[10];
  char gpsSpeedChar[10];
  char gpsCourseChar[10];

  dtostrf(gpsLat, 2, 6, gpsLatChar);
  Serial.println(gpsLatChar);
  dtostrf(gpsLng, 2, 6, gpsLngChar);
  Serial.println(gpsLngChar);
  dtostrf(gpsAlt, 2, 2, gpsAltChar);
  Serial.println(gpsAltChar);
  dtostrf(gpsSpeed, 2, 2, gpsSpeedChar);
  Serial.println(gpsSpeedChar);
  dtostrf(gpsCourse, 2, 2, gpsCourseChar);
  Serial.println(gpsCourseChar);

  int stringPosition = -1;

  // Latitude
  for (int x = 0; ; x++) {
    stringPosition++;
    char c = gpsLatChar[x];
    if (c == '\0') break;
    txString[stringPosition] = c;
  }
  txString[stringPosition] = ',';
  //stringPosition++;

  // Longitude
  for (int x = 0; ; x++) {
    stringPosition++;
    char c = gpsLngChar[x];
    if (c == '\0') break;
    txString[stringPosition] = c;
  }
  txString[stringPosition] = ',';
  //stringPosition++;

  // Altitude
  for (int x = 0; ; x++) {
    stringPosition++;
    char c = gpsAltChar[x];
    if (c == '\0') break;
    txString[stringPosition] = c;
  }
  txString[stringPosition] = ',';
  //stringPosition++;

  // Speed
  for (int x = 0; ; x++) {
    stringPosition++;
    char c = gpsSpeedChar[x];
    if (c == '\0') break;
    txString[stringPosition] = c;
  }
  txString[stringPosition] = ',';
  //stringPosition++;

  // Course
  for (int x = 0; ; x++) {
    stringPosition++;
    char c = gpsCourseChar[x];
    if (c == '\0') break;
    txString[stringPosition] = c;
  }
  txString[stringPosition] = '\n';
  txString[stringPosition + 1] = '\0';

  for (int x = 0; ; x++) {
    char c = txString[x];
    if (c == '\0') break;
    Serial.write(c);
  }

  Serial.println("COMPLETE");

  while (true) {
    ;
  }
}
