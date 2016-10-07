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
   - GPS
   - SHT11
   - Light
   - Data logging
   - Heater
   - Cut-down
   - Buzzer
   - Change global variables to functions returning pointer arrays

   CONSIDERATIONS:
   - Descent/landing triggering
   - Wait until descent/landing to power GPRS
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

// Analog Pins
const int lightPin = A0;
const int gasPins[] = {A7, A8, A9, A10, A11, A12, A13, A14, A15};

// Constants
char log_file[] = "hab_log.txt";
const char smsTargetNum[] = "+12145635266";

// Global variables
float gpsLat, gpsLng, gpsAlt, gpsSpeed;
int gpsHdop, gpsAge, gpsCourse;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
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

  Serial.begin(9600); // Debug output
  Serial1.begin(19200); // GPRS communication
  Serial2.begin(4800);  // GPS

  Serial.print("Initializing SD card...");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }
  Serial.println("success.");

  initSensors();
}

void loop() {
  // Stuff & Things
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
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("ERROR: not getting any GPS data!");
    // dump the stream to Serial
    Serial.println("GPS stream dump:");
    while (true) // infinite loop
      if (Serial1.available() > 0) // any data coming in?
        Serial.write(Serial1.read());
  }
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
  lightVal = analogRead(lightPin);
}

void logData(String dataString) {
  if (!logFile.open(log_file, O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("Opening log file for write failed");
  }
  logFile.println(dataString);
  logFile.close();
}

void debugSerialPrint() {
  Serial.print("DOF Accel: ");
  Serial.print(accelX); Serial.print(", ");
  Serial.print(accelY); Serial.print(", ");
  Serial.println(accelZ);
  Serial.print("DOF Gyro: ");
  Serial.print(gyroX); Serial.print(", ");
  Serial.print(gyroY); Serial.print(", ");
  Serial.println(gyroZ);
  Serial.print("DOF Mag: ");
  Serial.print(magX); Serial.print(", ");
  Serial.print(magY); Serial.print(", ");
  Serial.println(magZ);
  Serial.print("DOF AHRS: ");
  Serial.print(dofRoll); Serial.print(", ");
  Serial.print(dofPitch); Serial.print(", ");
  Serial.println(dofHeading);
  Serial.print("DOF Baro: ");
  Serial.print(dofPressure); Serial.print("hPa, ");
  Serial.print(dofTemp); Serial.print("C, ");
  Serial.print(dofAlt); Serial.println("m");
  Serial.print("Gas Sensors: ");
  for (int x = 0; x < 8; x++) {
    Serial.print(gasValues[x]); Serial.print(", ");
  }
  Serial.println(gasValues[8]);
  Serial.print("SHT11: ");
  Serial.print(shtTemp); Serial.print(" C, ");
  Serial.print(shtHumidity); Serial.println(" %RH");
  Serial.print("Light: "); Serial.println(lightVal);
}

