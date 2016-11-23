/*  NTX2 Radio Test Part 2

    Transmits data via RTTY with a checksum.

    Created 2012 by M0UPU as part of a UKHAS Guide on linking NTX2 Modules to Arduino.
    RTTY code from Rob Harrison Icarus Project.
    http://ukhas.org.uk
*/

#include <util/crc16.h>

const int radioPin = 9;

const char callsignHeader[] = "$$KG5CKI";

float gpsLat = 30.123456;
float gpsLng = 31.123456;
float gpsAlt = 123.12;
float gpsSpeed = 10.12;
float gpsCourse = 234.56;
float dofAlt = 123.34

void setup() {
  pinMode(radioPin, OUTPUT);
}

void loop() {
  rttyProcessTx();
  delay(2000);
}

void rttyProcessTx() {
  char rttyTxString[100];
  char gpsLatChar[10];
  char gpsLngChar[10];
  char gpsAltChar[10];
  char gpsSpeedChar[10];
  char gpsCourseChar[10];
  char dofAltChar[10];
  char commaChar[] = ",";

  sprintf(rttyTxString, callsignHeader);
  strcat(rttyTxString, commaChar);
  dtostrf(gpsLat, 2, 6, gpsLatChar);
  strcat(rttyTxString, gpsLatChar);
  strcat(rttyTxString, commaChar);
  dtostrf(gpsLng, 2, 6, gpsLngChar);
  strcat(rttyTxString, gpsLngChar);
  strcat(rttyTxString, commaChar);
  dtostrf(gpsAlt, 2, 2, gpsAltChar);
  strcat(rttyTxString, gpsAltChar);
  strcat(rttyTxString, commaChar);
  dtostrf(gpsSpeed, 2, 2, gpsSpeedChar);
  strcat(rttyTxString, gpsSpeedChar);
  strcat(rttyTxString, commaChar);
  dtostrf(gpsCourse, 2, 2, gpsCourseChar);
  strcat(rttyTxString, gpsCourseChar);
  strcat(rttyTxString, commaChar);
  dtostrf(dofAlt, 2, 2, dofAltChar);
  strcat(rttyTxString, dofAltChar);

  unsigned int CHECKSUM = rttyCRC16Checksum(rttyTxString);
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(rttyTxString, checksum_str);

  rttyTxData(rttyTxString);
}

void rttyTxData (char * string) {
  /* Simple function to sent a char at a time to
     ** rttyTxByte function.
    ** NB Each char is one byte (8 Bits)
  */
  char c;

  c = *string++;

  while (c != '\0') {
    rttyTxByte (c);
    c = *string++;
  }
}

void rttyTxByte (char c) {
  /* Simple function to sent each bit of a char to
    ** rttyTxBit function.
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and
    ** proceded with a 1. 0 = Start bit; 1 = Stop bit
    **
  */
  rttyTxBit (0); // Start bit

  // Send bits for for char LSB first
  for (int i = 0; i < 7; i++) { // Change this here 7 or 8 for ASCII-7 / ASCII-8
    if (c & 1) rttyTxBit(1);
    else rttyTxBit(0);

    c = c >> 1;
  }
  rttyTxBit (1); // Stop bit
  rttyTxBit (1); // Stop bit
}

void rttyTxBit (int bit) {
  if (bit) digitalWrite(radioPin, HIGH);
  else digitalWrite(radioPin, LOW);

  delayMicroseconds(3370); // 300 baud
  //delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
  //delayMicroseconds(10150); // 20150 doesn't work...16383 largest value Arduino can handle
}

uint16_t rttyCRC16Checksum (char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}
