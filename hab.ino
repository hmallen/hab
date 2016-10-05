/*
 * High-Altitude Balloon
 * 
 * Features:
 * - Sensor data acquisition
 * - Relay control
 * - SD data logging
 * - SMS notifications
 * 
 * Sensors:
 * - GPS
 * - Gas Sensors
 * - Temp/Humidity (SHT11)
 * - Light
 * - Accelerometer/Gyroscope/Magnetometer
 * - Barometer (Altitude/Temperature)
 * 
 * Relay Control:
 * #1 --> Pin 7
 * #2 --> Pin 6
 * #3 --> Pin 5
 * #4 --> Pin 4
 */

// Libraries
#include <SdFat.h>
//#include <SoftwareSerial.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <Wire.h>

// Definitions

// Digital Pins

// Analog Pins

// Constants
const int chipSelect = SS;

String log_file = "hab_log.txt";

// Initializers


void setup() {
  // PIN MODES AND DIGITAL WRITES TO INITIALIZE
  
  Serial.begin(9600);

  Serial.print("Initializing SD card...");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }
  Serial.println("success.");
}

void loop() {
  // Stuff & Things
}

void logData(String dataString) {
  if (!logFile.open(log_file, O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("Opening log file for write failed");
  }
  logFile.println(dataString);
  logFile.close();
}
