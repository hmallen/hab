/*
   High-Altitude Balloon

   Features:
   - Sensor data acquisition
   - Relay control
   - SD data logging
   - SMS notifications

   Sensors:
   - GPS
   - Gas Sensors
   - Temp/Humidity (SHT11)
   - Light
   - Accelerometer/Gyroscope/Magnetometer
   - Barometer (Altitude/Temperature)

   Relay Control:
   #1 --> Pin 7
   #2 --> Pin 6
   #3 --> Pin 5
   #4 --> Pin 4

   Gas Sensors:
   MQ-2 --> A7
   MQ-3 --> A8
   MQ-4 --> A9
   MQ-5 --> A10
   MQ-6 --> A11
   MQ-7 --> A12
   MQ-8 --> A13
   MQ-9 --> A14
   MQ135 --> A15

   TO DO:
   - Consider setting sampling rate based on theoretical ascent rate
   - Integrate GPRS
   - Error logging (add new and change serial prints to log)
   - Heater
   - Cut-down
   - Buzzer
   - Change global variables to functions returning pointer arrays
   - Set internal time (or use GPS/GPRS)
   - Change read data functions to bools to monitor for any errors
   - Update "last known coordinates" from GPS data if changed

   CONSIDERATIONS:
   - Update Adafruit 1604 quickly and other sensors more slowly (Add bools to show when update occurs?)
   - Descent/landing triggering
   - Wait until descent/landing to power GPRS
   OR
   - **** Power-off gas sensors and power-on GPRS at same time ****
   - Utilize TinyGPS++ libraries to calculate distance and course from home
*/

// Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SdFat.h>
#include <SHT1x.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>

// Definitions
#define gpsHdopThreshold 200  // Change to 150 for live conditions
#define auxDataInterval 10000 // Update interval (ms) for data other than that from Adafruit 1604

// Digital Pins
const int chipSelect = SS;
const int shtData = 2;
const int shtClock = 3;
const int relayPin1 = 7;
const int relayPin2 = 6;
const int relayPin3 = 5;
const int relayPin4 = 4;
const int smsPowerPin = 22;
const int gpsReadyLED = 23;
const int programStartPin = 24;
const int programStartLED = 25;

// Analog Pins
const int lightPin = A0;
const int gasPins[] = {A7, A8, A9, A10, A11, A12, A13, A14, A15};

// Constants
const char log_file[] = "hab_log.txt";
const char debug_file[] = "hab_debug.txt";
const char smsTargetNum[] = "+12145635266";

// Global variables
float gpsLat, gpsLng, gpsAlt, gpsSpeed;
uint32_t gpsDate, gpsTime, gpsSats, gpsCourse;
int32_t gpsHdop;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;  // Can this be a const float??
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float dofRoll, dofPitch, dofHeading;
float dofPressure, dofTemp, dofAlt;
int gasValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float shtTemp, shtHumidity;
int lightVal;

// Initializers
SdFat sd;
SdFile logFile;
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
SHT1x sht(shtData, shtClock);
TinyGPSPlus gps;

boolean debugMode = true;

void initSensors() {
  // Adafruit 1604
  if (!accel.begin()) {
    Serial.println("No LSM303 detected...Check your wiring!");
    while (true);
  }
  if (!mag.begin()) {
    Serial.println("No LSM303 detected...Check your wiring!");
    while (true);
  }
  if (!gyro.begin()) {
    Serial.print("No L3GD20 detected...Check your wiring or I2C address!");
    while (true);
  }
  if (!bmp.begin()) {
    Serial.println("No BMP180 detected...Check your wiring!");
    while (true);
  }

  for (int x = 0; x < 3; x++) {
    float initShtTempC = sht.readTemperatureC();
    delay(100);
    float initShtTempF = sht.readTemperatureF();
    delay(100);
    float initShtHumidity = sht.readHumidity();
    delay(100);
  }

  Serial.println("Waiting for sufficient GPS HDOP.");
  while (true) {
    readGps();
    Serial.print("Sats: "); Serial.print(gpsSats);
    Serial.print(" / ");
    Serial.print("HDOP: "); Serial.println(gpsHdop);
    if (gpsHdop < gpsHdopThreshold && gpsHdop != 0) break;
    delay(1000);
  }
  Serial.println("GPS lock attained.");
  digitalWrite(gpsReadyLED, HIGH);
}

void powerSMS() {
  digitalWrite(smsPowerPin, HIGH);
  delay(500);
  digitalWrite(smsPowerPin, LOW);
}

void setup() {
  pinMode(chipSelect, OUTPUT);
  pinMode(relayPin1, OUTPUT); digitalWrite(relayPin1, LOW);
  pinMode(relayPin2, OUTPUT); digitalWrite(relayPin2, LOW);
  pinMode(relayPin3, OUTPUT); digitalWrite(relayPin3, LOW);
  pinMode(relayPin4, OUTPUT); digitalWrite(relayPin4, LOW);
  pinMode(smsPowerPin, OUTPUT); digitalWrite(smsPowerPin, LOW);
  pinMode(gpsReadyLED, OUTPUT); digitalWrite(gpsReadyLED, LOW);
  pinMode(programStartLED, OUTPUT); digitalWrite(programStartLED, LOW);
  pinMode(programStartPin, INPUT_PULLUP);

  Serial.begin(9600); // Debug output
  Serial1.begin(19200); // GPRS communication
  Serial2.begin(4800);  // GPS

  Serial.println();
  Serial.print("Initializing SD card...");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }
  Serial.println("complete.");

  Serial.println("Initializing sensors.");
  initSensors();
  Serial.println("Sensor initialization complete.");
  Serial.println();
  Serial.print("Waiting for program start trigger...");
  while (true) {
    if (digitalRead(programStartPin) == 1) break;
    delay(100);
  }
  digitalWrite(programStartLED, HIGH);
  Serial.println("starting program.");
  Serial.println();
}

void loop() {
  for (unsigned long startTime = millis(); (millis() - startTime) < auxDataInterval; ) {
    readAda1604();
    if (debugMode) {
      debugDofPrint();
    }
    delay(1000);
  }
  Serial.println();

  readGps();
  readGas();
  readSht();
  readLight();

  if (!debugMode) logData();
  else debugDataPrint();
}

void readAda1604() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t gyro_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  gyro.getEvent(&gyro_event);
  mag.getEvent(&mag_event);
  bmp.getEvent(&bmp_event);

  accelX = accel_event.acceleration.x;
  accelY = accel_event.acceleration.y;
  accelZ = accel_event.acceleration.z;

  gyroX = gyro_event.gyro.x;
  gyroY = gyro_event.gyro.y;
  gyroZ = gyro_event.gyro.z;

  magX = mag_event.magnetic.x;
  magY = mag_event.magnetic.y;
  magZ = mag_event.magnetic.z;

  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
    dofRoll = orientation.roll;
    dofPitch = orientation.pitch;
    dofHeading = orientation.heading;
  }

  if (bmp_event.pressure) {
    float temperature;
    bmp.getTemperature(&temperature);
    dofPressure = bmp_event.pressure;
    dofTemp = temperature;
    dofAlt = bmp.pressureToAltitude(seaLevelPressure, dofPressure, dofTemp);
  }
}

void readGps() {
  unsigned long startTime = millis();

  while ((millis() - startTime) < 1000) {
    while (Serial2.available()) {
      gps.encode(Serial2.read());
    }
  }
  gpsDate = gps.date.value();
  gpsTime = gps.time.value();
  gpsSats = gps.satellites.value();
  gpsHdop = gps.hdop.value();
  gpsLat = gps.location.lat();
  gpsLng = gps.location.lng();
  gpsAlt = gps.altitude.meters();
  gpsSpeed = gps.speed.mps();
  gpsCourse = gps.course.deg();
}
void readGas() {
  int gasPinLength = sizeof(gasPins) / 2;
  for (int x = 0; x < gasPinLength; x++) {
    gasValues[x] = analogRead(gasPins[x]);
    delay(10);
  }
}

void readSht() {
  shtTemp = sht.readTemperatureC();
  shtHumidity = sht.readHumidity();
}

void readLight() {
  int lightValRaw = 1023 - analogRead(lightPin);
  lightVal = map(lightValRaw, 0, 1023, 0, 100);
}

void logData() {
  if (!logFile.open(log_file, O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("Opening debug log for write failed.");
  }
  logFile.print(gpsDate); logFile.print(",");
  logFile.print(gpsTime); logFile.print(",");
  logFile.print(gpsLat); logFile.print(",");
  logFile.print(gpsLng); logFile.print(",");
  logFile.print(gpsSats); logFile.print(",");
  logFile.print(gpsHdop); logFile.print(",");
  logFile.print(gpsAlt); logFile.print(",");
  logFile.print(gpsSpeed); logFile.print(",");
  logFile.print(gpsCourse); logFile.print(",");
  logFile.print(accelX); logFile.print(",");
  logFile.print(accelY); logFile.print(",");
  logFile.print(accelZ); logFile.print(",");
  logFile.print(gyroX); logFile.print(",");
  logFile.print(gyroY); logFile.print(",");
  logFile.print(gyroZ); logFile.print(",");
  logFile.print(magX); logFile.print(",");
  logFile.print(magY); logFile.print(",");
  logFile.print(magZ); logFile.print(",");
  logFile.print(dofRoll); logFile.print(",");
  logFile.print(dofPitch); logFile.print(",");
  logFile.print(dofHeading); logFile.print(",");
  logFile.print(dofPressure); logFile.print(",");
  logFile.print(dofTemp); logFile.print(",");
  logFile.print(dofAlt); logFile.print(",");
  for (int x = 0; x < 9; x++) {
    logFile.print(gasValues[x]); logFile.print(",");
  }
  logFile.print(shtTemp); logFile.print(",");
  logFile.print(shtHumidity); logFile.print(",");
  logFile.println(lightVal);
  logFile.close();
}

void logOther(String dataString) {
  if (!logFile.open(debug_file, O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("Opening debug log for write failed.");
  }
  logFile.println(dataString);
  logFile.close();
}

void debugDofPrint() {
  Serial.print(dofRoll); Serial.print("(Roll),");
  Serial.print(dofPitch); Serial.print("(Pitch),");
  Serial.print(dofHeading); Serial.print("(Heading) / ");
  Serial.print(dofPressure); Serial.print("hPa,");
  Serial.print(dofTemp); Serial.print("C,");
  Serial.print(dofAlt); Serial.println("m");
}

void debugDataPrint() {
  Serial.print("GPS: ");
  Serial.print(gpsDate); Serial.print(",");
  Serial.print(gpsTime); Serial.print(",");
  Serial.print(gpsLat); Serial.print(",");
  Serial.print(gpsLng); Serial.print(" ");
  Serial.print("Sats="); Serial.print(gpsSats); Serial.print(" ");
  Serial.print("HDOP="); Serial.print(gpsHdop); Serial.print(" ");
  Serial.print(gpsAlt); Serial.print("m ");
  Serial.print(gpsSpeed); Serial.print("m/s ");
  Serial.print(gpsCourse); Serial.println("deg[Course]");
  Serial.print("DOF Accel: ");
  Serial.print(accelX); Serial.print(",");
  Serial.print(accelY); Serial.print(",");
  Serial.println(accelZ);
  Serial.print("DOF Gyro: ");
  Serial.print(gyroX); Serial.print(",");
  Serial.print(gyroY); Serial.print(",");
  Serial.println(gyroZ);
  Serial.print("DOF Mag: ");
  Serial.print(magX); Serial.print(",");
  Serial.print(magY); Serial.print(",");
  Serial.println(magZ);
  Serial.print("DOF AHRS: ");
  Serial.print(dofRoll); Serial.print(",");
  Serial.print(dofPitch); Serial.print(",");
  Serial.println(dofHeading);
  Serial.print("DOF Baro: ");
  Serial.print(dofPressure); Serial.print("hPa,");
  Serial.print(dofTemp); Serial.print("C,");
  Serial.print(dofAlt); Serial.println("m");
  Serial.print("Gas Sensors: ");
  for (int x = 0; x < 8; x++) {
    Serial.print(gasValues[x]); Serial.print(",");
  }
  Serial.println(gasValues[8]);
  Serial.print("SHT11: ");
  Serial.print(shtTemp); Serial.print("C,");
  Serial.print(shtHumidity); Serial.println("%RH");
  Serial.print("Light: "); Serial.print(lightVal); Serial.println("%");
  Serial.println();
}
