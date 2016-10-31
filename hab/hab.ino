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
   #1 --> Pin 7 (Gas sensors)
   #2 --> Pin 6 (Internal payload heater)
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

   EEPROM Values:
   0 --> Setup complete
   1 --> Descent phase
   2 --> Landing phase

   TO DO:
   - Store launch coordinates in EEPROM
   - Turn-off all serial prints when debugMode set to false
   - Confirm that boolean argument in smsPower function actually necessary
   - Set digital read vals to HIGH/LOW for consistency
   - Test if SMS stores in buffer when no signal and sends when reconnected
   - Add check to confirm GPRS is powered on
   - Log incoming/outgoing SMS messages to SD card
   - Change digital pin order for uniformity
   - Consider setting sampling rate based on theoretical ascent rate
   - Cut-down
   - Change global variables to functions returning pointer arrays
   - Set home (launch site) GPS coordinates
   -- Utilize TinyGPS++ libraries to calculate distance and course from home

   CONSIDERATIONS:
   - Averaging ~3x GPS coordinates before setting launch site
   - Check launch coordinates to confirm within threshold distance of theoretical launch site
   - Check if system resets if on external power and serial (computer debugging) is unplugged
   - Rename relay pins to better indicate function
   - Gas sensor calibration
   - Possible to use SMS command to reset system or place into "recovery" mode?
   - TURN ON ROAMING BEFORE LIVE LAUNCH TO ENSURE PRESENCE OF GPRS NETWORK CONNECTION
   - Create LED flashes to indicate specific startup failure
   - Check if break in DOF ms5607 for other data affects reconstruction
   - Find function to confirm GPRS is connected to network
   - Use GMT for data logging but include convert function for human-friendly output
   - Update Adafruit 1604 quickly and other ms5607s more slowly (Add bools to show when update occurs?)
   -- Determine optimal update rate for each
   - Descent/landing triggering
   - Power-off gas ms5607s and power-on GPRS at same time

   LESSONS LEARNED:
   - I2C device failures (first observed w/ MS5607 CRC4 check fail) likely due to poor jumper/breadboard wiring
*/

// Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <EEPROM.h>
#include <MS5xxx.h>
#include <SdFat.h>
#include <SHT1x.h>
#include <SPI.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <Wire.h>

// Definitions
#define GPSHDOPTHRESHOLD 150  // Change to 150 for live conditions
#define GASSENSORWARMUP 300000  // Gas sensor warm-up time (ms)
#define DOFDATAINTERVAL 500 // Update interval (ms) for Adafruit 1604 data
#define AUXDATAINTERVAL 5000 // Update interval (ms) for data other than that from Adafruit 1604
#define GPSDATAINTERVAL 15000 // Update interval (ms) for GPS data updates
#define SMSPOWERDELAY 5000  // Delay between power up and sending of commands to GPRS
#define SERIALTIMEOUT 5000  // Time to wait for incoming GPRS serial data before time-out
#define BAROALTCHANGETHRESHOLD 5.0  // Negative altitude change (m) from altimeters to signify real altitude decrease
#define BAROPRESSCHANGETHRESHOLD 0.5 // Negative pressure change (hPa) from altimeters to signify real altitude decrease
#define GPSCHANGETHRESHOLD 5.0 // Negative altitude change (m) from GPS to signify real altitude decrease
#define HEATERTRIGGERTEMP 0.0  // Temperature (C) below which internal payload heater activated

//#define DAYLIGHTSAVINGS

#ifdef DAYLIGHTSAVINGS
const int gpsTimeOffset = -4;
#else
const int gpsTimeOffset = -5;
#endif

const bool debugMode = false;
const bool debugSmsOff = false;

// Digital Pins
const int chipSelect = SS;
const int shtData = 2;
const int shtClock = 3;
const int gasRelay = 7;  // Gas sensors
const int heaterRelay = 6;  // Internal payload heater
const int relayPin3 = 5;
const int relayPin4 = 4;
const int smsPowerPin = 22;
const int gpsPpsPin = 26;
const int gpsReadyLED = 23;
const int programStartPin = 24;
const int programStartLED = 25;
const int programReadyPin = 27;
const int heartbeatOutputPin = 28;
const int buzzerPin = 29;

// Analog Pins
const int lightPin = A0;
const int gasPins[] = {A7, A8, A9, A10, A11, A12, A13, A14, A15};

// Constants
const char dof_log_file[] = "dof_log.txt";
const char aux_log_file[] = "aux_log.txt";
const char gps_log_file[] = "gps_log.txt";
const char other_log_file[] = "other_log.txt";
const char debug_log_file[] = "debug_log.txt";
const char smsTargetNum[] = "+12145635266";

// Global variables
bool gpsTimeSet = false;
bool smsReadyReceived = false;
bool smsFirstFlush = false;
bool smsMarkFlush = false;
String smsCommandText = "";
bool gpsFix = false;
float gpsLaunchLat, gpsLaunchLng;
float gpsLat, gpsLng, gpsAlt, gpsSpeed;
uint32_t gpsDate, gpsTime, gpsSats, gpsCourse;
float gpsDistAway, gpsCourseTo;
String gpsCourseToText;
int32_t gpsHdop;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;  // Can this be a const float??
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float dofRoll, dofPitch, dofHeading;
float dofPressure, dofTemp, dofAlt;
float ms5607Temp, ms5607Press;
float gasValues[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float gasValuesLast[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float shtTemp, shtHumidity;
float lightVal;

// Old values to process with checkChange() function
float gpsLatLast, gpsLngLast, gpsAltLast, ms5607PressLast, dofAltLast;
int gpsChanges = 0;
int ms5607Changes = 0;
int dofChanges = 0;


// EEPROM Switches
bool setupComplete = false; // EEPROM #0
bool descentPhase = false;  // EEPROM #1
bool landingPhase = false;  // EEPROM #2

int loopCount = 1;
int dofLoopCount = 1;
int auxLoopCount = 1;
int gpsLoopCount = 1;

// Initializers
SdFat sd;
SdFile logFile;
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
MS5xxx ms5607(&Wire);
SHT1x sht(shtData, shtClock);
TinyGPSPlus gps;

bool dofValid, ms5607Valid, gpsValid, gasValid, shtValid, lightValid;
int ada1604Failures = 0;
int gpsFailures = 0;
int ms5607Failures = 0;
int gasFailures = 0;
int shtFailures = 0;
int lightFailures = 0;

void initSensors() {
  // Adafruit 1604
  Serial.print(" - DOF...");
  if (!accel.begin()) {
    Serial.println("No LSM303 detected.");
    startupFailure();
  }
  if (!mag.begin()) {
    Serial.println("No LSM303 detected.");
    startupFailure();
  }
  if (!gyro.begin()) {
    Serial.println("No L3GD20 detected.");
    startupFailure();
  }
  if (!bmp.begin()) {
    Serial.println("No BMP180 detected.");
    startupFailure();
  }
  Serial.println("success.");

  Serial.print(" - MS5607...");
  if (ms5607.connect() > 0) {
    Serial.println("No MS5607 detected.");
    delay(500);
    setup();  // CHANGE TO STARTUPFAILURE() FUNCTION?
  }
  Serial.println("success.");

  Serial.print(" - SHT11...");
  for (int x = 0; x < 3; x++) {
    float initShtTempC = sht.readTemperatureC();
    delay(100);
    float initShtTempF = sht.readTemperatureF();
    delay(100);
    float initShtHumidity = sht.readHumidity();
    delay(100);
  }
  Serial.println("success.");
}

void initGps() {
  bool gpsPower = false;
  bool gpsPpsVal;
  int gpsPpsPulses;

  Serial.print("Checking for GPS power...");
  while (gpsPower == false) {
    gpsPpsPulses = 0;
    for (unsigned long startTime = millis(); (millis() - startTime) < 1000; ) {
      gpsPpsVal = digitalRead(gpsPpsPin);
      if (gpsPpsVal == true) {
        gpsPpsPulses++;
        //Serial.print("GPS PULSES: "); Serial.println(gpsPpsPulses);
      }
      delay(100);
    }
    if (gpsPpsPulses < 5) gpsPower = true;
  }
  Serial.println("confirmed.");

  Serial.print("Waiting for GPS fix...");
  while (gpsFix == false) {
    gpsPpsPulses = 0;
    for (unsigned long startTime = millis(); (millis() - startTime) < 6000; ) {
      gpsPpsVal = digitalRead(gpsPpsPin);
      if (gpsPpsVal == true) {
        gpsPpsPulses++;
        //Serial.print("GPS PULSES: "); Serial.println(gpsPpsPulses);
      }
      delay(100);
    }
    if (gpsPpsPulses >= 5) gpsFix = true;
  }
  Serial.println("attained.");

  Serial.print("Waiting for sufficient HDOP...");
  readGps();
  while (gpsHdop >= GPSHDOPTHRESHOLD) {
    delay(1000);
    readGps();
  }
  Serial.println("attained.");

  readGps();
  gpsLaunchLat = gpsLat;
  gpsLaunchLng = gpsLng;

  EEPROM.put(10, gpsLaunchLat);
  EEPROM.put(20, gpsLaunchLng);

  if (debugMode) {
    Serial.print("Launch Coordinates: ");
    Serial.print(gpsLaunchLat); Serial.print(", ");
    Serial.println(gpsLaunchLng);
  }
  Serial.println();

  digitalWrite(gpsReadyLED, HIGH);
}

void smsPower(bool powerState) {
  // Power on GPRS and set proper modes for operation
  if (powerState == true) {
    digitalWrite(smsPowerPin, HIGH);
    delay(500);
    digitalWrite(smsPowerPin, LOW);

    delay(SMSPOWERDELAY);

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

void smsConfirmReady() {
  String smsMessageRaw = "";
  while (smsReadyReceived == false) {
    if (!Serial1.available()) {
      while (!Serial1.available()) {
        digitalWrite(programStartLED, HIGH);
        delay(100);
        digitalWrite(programStartLED, LOW);
        delay(100);
      }
    }
    if (Serial1.available()) {
      if (smsFirstFlush == false) {
        smsFlush();
        smsFirstFlush = true;
      }
      else {
        while (Serial1.available()) {
          char c = Serial1.read();
          smsMessageRaw += c;
          delay(10);
        }
        smsHandler(smsMessageRaw, false, true);
      }
    }
  }
}

void setup() {
  pinMode(chipSelect, OUTPUT);
  pinMode(gasRelay, OUTPUT); digitalWrite(gasRelay, HIGH);  // Gas sensors
  pinMode(heaterRelay, OUTPUT); digitalWrite(heaterRelay, LOW); // Payload heater
  pinMode(relayPin3, OUTPUT); digitalWrite(relayPin3, LOW);
  pinMode(relayPin4, OUTPUT); digitalWrite(relayPin4, LOW);
  pinMode(smsPowerPin, OUTPUT); digitalWrite(smsPowerPin, LOW);
  pinMode(gpsReadyLED, OUTPUT); digitalWrite(gpsReadyLED, LOW);
  pinMode(programStartLED, OUTPUT); digitalWrite(programStartLED, LOW);
  pinMode(programReadyPin, OUTPUT); digitalWrite(programReadyPin, LOW);
  pinMode(heartbeatOutputPin, OUTPUT); digitalWrite(heartbeatOutputPin, LOW);
  pinMode(buzzerPin, OUTPUT); digitalWrite(buzzerPin, LOW);
  pinMode(gpsPpsPin, INPUT_PULLUP);
  pinMode(programStartPin, INPUT_PULLUP);

  Serial.begin(9600); // Debug output
  Serial1.begin(19200); // GPRS communication
  Serial2.begin(9600);  // GPS communication

  if (debugMode) {
    Serial.println();
    Serial.println(F("---------------"));
    Serial.println(F("---- SETUP ----"));
    Serial.println(F("---------------"));
    Serial.println();
  }

  setupComplete = EEPROM.read(0);
  if (setupComplete) {
    descentPhase = EEPROM.read(1);
    if (descentPhase) landingPhase = EEPROM.read(2);

    EEPROM.get(10, gpsLaunchLat);
    EEPROM.get(20, gpsLaunchLng);

    if (descentPhase || landingPhase) digitalWrite(gasRelay, LOW);

    smsFlush();

    if (debugMode) {
      Serial.println("System rebooted by watchdog timer or manual reset.");
      Serial.println();
    }

    if (digitalRead(programStartPin) == 0) {
      Serial.print("Clearing EEPROM...");
      for (int x = 0; x < 3; x++) {
        EEPROM.update(x, 0);
      }
      for (int x = 10; x < 14; x++) {
        EEPROM.update(x, 0);
      }
      for (int x = 20; x < 24; x++) {
        EEPROM.update(x, 0);
      }
      Serial.println("complete.");
      Serial.println();

      initGps();
    }

    digitalWrite(gpsReadyLED, HIGH);
  }
  else {
    unsigned int startTime = millis();  // Begin gas sensor warmup timer

    initGps();

    Serial.print("Powering GPRS...");
    smsPower(true);
    if (!debugMode) delay(SMSPOWERDELAY);
    Serial.println("complete.");
    Serial.println();

    for (int x = 0; x < 2; x++) {
      digitalWrite(programStartLED, HIGH);
      delay(100);
      digitalWrite(programStartLED, LOW);
      delay(100);
    }

    if (!debugSmsOff) {
      Serial.print("Sending SMS confirmation request...");

      Serial1.print("AT + CMGS = \""); Serial1.print(smsTargetNum); Serial1.println("\"");
      delay(100);
      Serial1.println("Reply with \"Ready\" to continue.");
      delay(100);
      Serial1.println((char)26);
      delay(100);
      smsFlush();
      smsMarkFlush = true;

      Serial.println("complete.");

      Serial.print("Waiting for SMS ready message...");
      smsConfirmReady();
      Serial.println("received.");
      Serial.println();
    }

    //smsPower(false);

    for (int x = 0; x < 2; x++) {
      digitalWrite(programStartLED, HIGH);
      delay(100);
      digitalWrite(programStartLED, LOW);
      delay(100);
    }

    Serial.print("Warming-up gas sensors...");
    if (!debugMode) {
      while ((millis() - startTime) < GASSENSORWARMUP) {
        ;
      }
    }
    Serial.println("complete.");
    Serial.println();
  }

  Serial.println("Initializing sensors:");
  initSensors();
  Serial.println("Complete.");
  Serial.println();

  Serial.print("Initializing SD card...");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt(); // CHANGE TO PREVENT PROGRAM STALL
  }
  Serial.println("complete.");
  Serial.println();

  if (digitalRead(programStartPin) == 0) {
    Serial.print("Toggle start switch to continue...");
    while (digitalRead(programStartPin) == 0) {
      digitalWrite(programStartLED, HIGH);
      delay(500);
      digitalWrite(programStartLED, LOW);
      delay(500);
    }
    Serial.println("starting program.");
    Serial.println();
  }

  if (debugMode) {
    Serial.println(F("-----------------"));
    Serial.println(F("---- PROGRAM ----"));
    Serial.println(F("-----------------"));
    Serial.println();
  }

  EEPROM.update(0, 1);
  digitalWrite(programReadyPin, HIGH);
  digitalWrite(programStartLED, HIGH);
}

void loop() {
  if (debugMode) {
    Serial.print("Loop #: "); Serial.println(loopCount);
    Serial.println();
  }
  unsigned long loopStart = millis();
  //for (unsigned long startTimeGps = millis(); (millis() - startTimeGps) < GPSDATAINTERVAL; ) {
  //for (unsigned long startTimeAux = millis(); (millis() - startTimeAux) < AUXDATAINTERVAL; ) {
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 10; y++) {
      unsigned long startTimeDof = millis();
      dofValid = readAda1604();
      if (!dofValid) {
        ada1604Failures++;
        if (debugMode) {
          Serial.print("Failed to read DOF. Count=");  // WRITE TO LOG FILE INSTEAD
          Serial.println(ada1604Failures);
        }
        else {

        }
      }
      Serial.print("DOF Loop #: "); Serial.println(dofLoopCount);

      if (debugMode) debugDofPrint();
      else logData("dof");
      delay(DOFDATAINTERVAL);
      dofLoopCount++;

      if (debugMode) Serial.print("Time(DOF): "); Serial.println((millis() - startTimeDof));
      heartbeat();
    }
    if (debugMode) {
      Serial.println();
      Serial.print("Time(Aux-START): "); Serial.println((millis() - loopStart));
    }

    ms5607Valid = readMS5607();
    if (!ms5607Valid) {
      ms5607Failures++;
      if (debugMode) {
        Serial.print("Failed to read MS5607. Count=");  // WRITE TO LOG FILE INSTEAD
        Serial.println(ms5607Failures);
      }
      else logDebug("Failed to read MS5607. Count=" + String(ms5607Failures));
      for (int x = 0; x < 5; x++) {
        if (readMS5607()) break;
        else {
          ms5607Failures++;
          if (debugMode) {
            Serial.print("Failed to read MS5607. Count=");  // WRITE TO LOG FILE INSTEAD
            Serial.println(ms5607Failures);
          }
          else logDebug("Failed to read MS5607. Count=" + String(ms5607Failures));
          ms5607Temp = 0.0;
          ms5607Press = 0.0;
        }
        delay(100);
      }
    }

    if (!descentPhase || !landingPhase) {
      gasValid = readGas();
      if (!gasValid) {
        gasFailures++;
        if (debugMode) {
          Serial.print("Failed to read gas sensors. Count=");  // WRITE TO LOG FILE INSTEAD
          Serial.println(gasFailures);
        }
        else logDebug("Failed to read gas sensors. Count=" + String(gasFailures));
      }
    }

    shtValid = readSht();
    if (!readSht()) {
      shtFailures++;
      if (debugMode) {
        Serial.print("Failed to read SHT11. Count=");  // WRITE TO LOG FILE INSTEAD
        Serial.println(shtFailures);
      }
      else logDebug("Failed to read SHT11. Count=" + String(shtFailures));
    }

    lightValid = readLight();
    if (!readLight()) {
      lightFailures++;
      if (debugMode) {
        Serial.print("Failed to read light sensor. Count=");  // WRITE TO LOG FILE INSTEAD
        Serial.println(lightFailures);
      }
      else logDebug("Failed to read light sensor. Count=" + String(lightFailures));
    }

    //Serial.println();
    Serial.print("Aux Loop #: "); Serial.println(auxLoopCount);
    auxLoopCount++;

    if (debugMode) debugAuxPrint();
    else logData("aux");
    if (debugMode) {
      Serial.print("Time(Aux-END): "); Serial.println((millis() - loopStart));
      Serial.println();
    }

    heartbeat();
  }

  gpsValid = readGps();
  if (!readGps()) {
    gpsFailures++;
    if (debugMode) {
      Serial.print("Failed to read GPS. Count=");  // WRITE TO LOG FILE INSTEAD
      Serial.println(gpsFailures);
    }
    else logDebug("Failed to read GPS. Count=" + String(gpsFailures));
    gpsLat = gpsLatLast;
    gpsLng = gpsLngLast;
  }
  Serial.print("GPS Loop #: "); Serial.println(gpsLoopCount);
  gpsLoopCount++;

  if (debugMode) debugGpsPrint();
  else logData("gps");
  if (debugMode) {
    Serial.print("Time(GPS): "); Serial.println(millis() - loopStart);
    Serial.println();
  }

  if (smsMarkFlush == true) {
    Serial.print("Flushing GPRS buffer...");
    smsFlush();
    Serial.println("complete.");
    Serial.println();
    smsMarkFlush = false;
  }

  if (Serial1.available()) {
    Serial.println("Incoming GPRS serial data.");
    String smsMessageRaw = "";
    while (Serial1.available()) {
      char c = Serial1.read();
      smsMessageRaw += c;
      delay(10);
    }
    Serial.print("Processing GPRS data...");
    smsHandler(smsMessageRaw, true, false);
    Serial.println("complete.");
    Serial.println();
  }

  if (smsCommandText != "") smsSendConfirmation();

  if (loopCount == 1 && !descentPhase && !landingPhase) {
    if (!debugSmsOff) smsMenu();
  }
  else {
    checkChange();
    if (debugMode) {
      Serial.print("GPS Changes: "); Serial.println(gpsChanges);
      Serial.print("DOF Changes: "); Serial.println(dofChanges);
      Serial.print("MS5607 Changes: "); Serial.println(ms5607Changes);
      Serial.println();
    }
  }

  heartbeat();
  loopCount++;
}

bool readAda1604() {
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
  else return false;

  if (bmp_event.pressure) {
    float temperature;
    bmp.getTemperature(&temperature);
    dofPressure = bmp_event.pressure;
    dofTemp = (float)temperature;
    dofAlt = bmp.pressureToAltitude(seaLevelPressure, dofPressure, dofTemp);
  }
  else return false;

  return true;
}

bool readGps() {
  unsigned long startTime = millis();

  while ((millis() - startTime) < 1000) {
    while (Serial2.available()) {
      gps.encode(Serial2.read());
    }
  }

  if (gps.location.isUpdated()) {
    gpsDate = gps.date.value();
    gpsTime = gps.time.value();
    gpsSats = gps.satellites.value();
    gpsHdop = gps.hdop.value();
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
    gpsAlt = gps.altitude.meters();
    gpsSpeed = gps.speed.mps();
    gpsCourse = gps.course.deg();

    if (gpsTimeSet == false) {
      unsigned long age;
      uint8_t Month = gps.date.month();
      uint8_t Day = gps.date.day();
      uint8_t Hour = gps.time.hour();
      uint8_t Minute = gps.time.minute();
      uint8_t Second = gps.time.second();

      uint16_t yearRaw = gps.date.year();
      String yearStringRaw = String(yearRaw);
      int yearLength = yearStringRaw.length();

      if (yearLength == 4 && age < 500) {
        String yearString;
        for (int x = 2; x < 4; x++) {
          char c = yearStringRaw.charAt(x);
          yearString += c;
        }
        int Year = yearString.toInt();

        setTime(Hour, Minute, Second, Day, Month, Year);
        adjustTime(gpsTimeOffset * SECS_PER_HOUR);
        gpsTimeSet = true;
      }
    }

    if (loopCount > 1) {
      gpsDistAway =
        gps.distanceBetween(
          gpsLat,
          gpsLng,
          gpsLaunchLat,
          gpsLaunchLng);
      gpsCourseTo =
        gps.courseTo(
          gpsLat,
          gpsLng,
          gpsLaunchLat,
          gpsLaunchLng);

      gpsCourseToText = String(gps.cardinal(gpsCourseTo));
    }
    return true;
  }
  else return false;
}

bool readMS5607() {
  for (int x = 0; x < 3; x++) {
    ms5607.ReadProm();
    ms5607.Readout();

    ms5607Temp = ms5607.GetTemp() / 100.0;
    ms5607Press = ms5607.GetPres() / 100.0;

    if ((-100.0 <= ms5607Temp <= 100.0) && (0.0 <= ms5607Press <= 1100.0)) return true;

    delay(100);
  }
  return false;
}

bool readGas() {
  int gasPinLength = sizeof(gasPins) / 2;
  for (int x = 0; x < gasPinLength; x++) {
    gasValuesLast[x] = gasValues[x];
  }

  for (int x = 0; x < gasPinLength; x++) {
    gasValues[x] = (float)map(analogRead(gasPins[x]), 0, 1023, 0, 1000) / 10.0;
    delay(10);
  }

  if (gasValues != gasValuesLast) return true;
  else return false;
}

bool readSht() {
  shtTemp = sht.readTemperatureC();
  shtHumidity = sht.readHumidity();

  if (-100.0 <= shtTemp <= 100.0 && 0.0 <= shtHumidity <= 100.0) return true;
  else return false;
}

bool readLight() {
  lightVal = (float)map((1023 - analogRead(lightPin)), 0, 1023, 0, 1000) / 10.0;
  if (0.0 <= lightVal <= 100.0) return true;
  else return false;
}

void checkChange() {
  static bool heaterStatus = digitalRead(heaterRelay);
  float gpsAltChange = gpsAltLast - gpsAlt;
  float dofAltChange = dofAltLast - dofAlt;
  float ms5607PressChange = ms5607Press - ms5607PressLast;

  if (dofTemp <= HEATERTRIGGERTEMP && heaterStatus == false) {
    digitalWrite(heaterRelay, HIGH);
    heaterStatus = true;
    logDebug("Heater activated @ " + String(dofTemp));
  }
  else if (dofTemp > HEATERTRIGGERTEMP && heaterStatus == true) {
    digitalWrite(heaterRelay, LOW);
    heaterStatus = false;
    logDebug("Heater inactivated @ " + String(dofTemp));
  }

  if (!descentPhase) {
    if (gpsAltChange <= 0.0) gpsChanges = 0;
    else if (gpsAltChange > GPSCHANGETHRESHOLD) gpsChanges++;

    if (dofAltChange <= 0.0) dofChanges = 0;
    else if (dofAltChange > BAROALTCHANGETHRESHOLD) dofChanges++;

    if (ms5607PressChange <= 0.0) ms5607Changes = 0;
    else if (ms5607PressChange > BAROPRESSCHANGETHRESHOLD) ms5607Changes++;

    if (gpsChanges >= 10) descentPhase = true;
    else if (dofChanges >= 10) descentPhase = true;
    else if (gpsChanges >= 5 && dofChanges >= 5) descentPhase = true;
    else if (gpsChanges >= 3 && dofChanges >= 3 && ms5607Changes >= 3) descentPhase = true;

    if (descentPhase) {
      EEPROM.write(1, 1);
      Serial.println();
      Serial.println("Descent phase triggered.");
      Serial.println();

      gpsChanges = 0;
      dofChanges = 0;
      ms5607Changes = 0;

      int gasPinLength = sizeof(gasPins) / 2;
      for (int x = 0; x < gasPinLength; x++) {
        gasValues[x] = 0.0;
      }

      digitalWrite(gasRelay, LOW);  // Shut-off gas sensors to save power
    }
  }

  else if (!landingPhase) {
    if (gpsAltChange > GPSCHANGETHRESHOLD) gpsChanges = 0;
    else if (gpsAltChange <= GPSCHANGETHRESHOLD) gpsChanges++;

    if (dofAltChange > BAROALTCHANGETHRESHOLD) dofChanges = 0;
    else if (dofAltChange <= BAROALTCHANGETHRESHOLD) dofChanges++;

    if (ms5607PressChange > BAROPRESSCHANGETHRESHOLD) ms5607Changes = 0;
    else if (ms5607PressChange <= BAROPRESSCHANGETHRESHOLD) ms5607Changes++;

    if (gpsChanges >= 10) landingPhase = true;
    else if (dofChanges >= 10) landingPhase = true;
    else if (gpsChanges >= 5 && dofChanges >= 5) landingPhase = true;
    else if (gpsChanges >= 3 && dofChanges >= 3 && ms5607Changes >= 3) landingPhase = true;

    if (landingPhase) {
      Serial.println();
      Serial.print("Landing phase triggered. Sending location via SMS...");
      Serial1.print("AT+CMGS=\""); Serial1.print(smsTargetNum); Serial1.println("\"");
      delay(100);
      Serial1.print("LANDING DETECTED "); Serial1.print(gpsDistAway); Serial1.print(" ");
      Serial1.print(gpsCourseToText); Serial1.print(" of launch site.");
      Serial1.print(": http://maps.google.com/maps?q=HAB@");
      Serial1.print(gpsLat, 6); Serial1.print(","); Serial1.print(gpsLng, 6);
      Serial1.println("&t=h&z=19&output=html");
      delay(100);
      Serial1.println((char)26);
      delay(100);
      smsFlush();
      smsMarkFlush = true;
      Serial.print("complete."); Serial.println();
    }
  }

  else {
    // ANY CHANGE CHECKS NECESSARY DURING LANDING PHASE
  }

  gpsAltLast = gpsAlt;
  dofAltLast = dofAlt;
  ms5607PressLast = ms5607Press;
}

void logData(String logType) {
  // GET RID OF SD.ERRORHALT() TO PREVENT PROGRAM FROM STALLING
  if (logType == "dof") {
    if (!logFile.open(dof_log_file, O_RDWR | O_CREAT | O_AT_END)) {
      if (debugMode) sd.errorHalt("Opening debug log for write failed.");
    }

    logFile.print(loopCount); logFile.print(",");
    logFile.print(month()); logFile.print(day()); logFile.print(year()); logFile.print("-");
    logFile.print(hour()); logFile.print(minute()); logFile.print(second()); logFile.print(",");
    logFile.print(dofValid); logFile.print(",");
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
    logFile.println(dofAlt);
  }

  else if (logType == "aux") {
    if (!logFile.open(aux_log_file, O_RDWR | O_CREAT | O_AT_END)) {
      if (debugMode) sd.errorHalt("Opening aux log for write failed.");
    }

    logFile.print(loopCount); logFile.print(",");
    logFile.print(month()); logFile.print(day()); logFile.print(year()); logFile.print("-");
    logFile.print(hour()); logFile.print(minute()); logFile.print(second()); logFile.print(",");
    logFile.print(ms5607Valid); logFile.print(",");
    logFile.print(ms5607Press); logFile.print(",");
    logFile.print(ms5607Temp); logFile.print(",");
    for (int x = 0; x < 9; x++) {
      logFile.print(gasValues[x]); logFile.print(",");
    }
    logFile.print(shtValid); logFile.print(",");
    logFile.print(shtTemp); logFile.print(",");
    logFile.print(shtHumidity); logFile.print(",");
    logFile.println(lightVal);
  }
  else if (logType == "gps") {
    if (!logFile.open(gps_log_file, O_RDWR | O_CREAT | O_AT_END)) {
      if (debugMode) sd.errorHalt("Opening GPS log for write failed.");
    }

    logFile.print(loopCount); logFile.print(",");
    logFile.print(month()); logFile.print(day()); logFile.print(year()); logFile.print("-");
    logFile.print(hour()); logFile.print(minute()); logFile.print(second()); logFile.print(",");
    logFile.print(gpsValid); logFile.print(",");
    logFile.print(gpsDate); logFile.print(",");
    logFile.print(gpsTime); logFile.print(",");
    logFile.print(gpsLat); logFile.print(",");
    logFile.print(gpsLng); logFile.print(",");
    logFile.print(gpsSats); logFile.print(",");
    logFile.print(gpsHdop); logFile.print(",");
    logFile.print(gpsAlt); logFile.print(",");
    logFile.print(gpsSpeed); logFile.print(",");
    logFile.print(gpsCourse); logFile.print(",");
    logFile.print(gpsDistAway); logFile.print(",");
    logFile.print(gpsCourseTo); logFile.print(",");
    logFile.println(gpsCourseToText);
  }
  else {
    if (debugMode) Serial.println("Unrecognized log type. Failed to write to SD.");
    else {
      logDebug("Unrecognized log type. Failed to write to SD.");
    }
  }

  logFile.flush();
  logFile.close();
}

void logDebug(String dataString) {
  // GET RID OF SD.ERRORHALT() TO PREVENT PROGRAM FROM STALLING
  if (!logFile.open(other_log_file, O_RDWR | O_CREAT | O_AT_END)) {
    if (debugMode) sd.errorHalt("Opening debug log for write failed.");
  }

  logFile.print(loopCount); logFile.print(",");
  logFile.print(month()); logFile.print(day()); logFile.print(year()); logFile.print("-");
  logFile.print(hour()); logFile.print(minute()); logFile.print(second()); logFile.print(",");
  logFile.println(dataString);
  logFile.flush();
  logFile.close();
}

void debugDofPrint() {
  Serial.print(dofRoll); Serial.print("(Roll), ");
  Serial.print(dofPitch); Serial.print("(Pitch), ");
  Serial.print(dofHeading); Serial.print("(Heading) / ");
  Serial.print(dofPressure); Serial.print("hPa, ");
  Serial.print(dofTemp); Serial.print("C, ");
  Serial.print(dofAlt); Serial.println("m");
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
}

void debugAuxPrint() {
  Serial.print("MS5607: ");
  Serial.print(ms5607Press); Serial.print("hPa, ");
  Serial.print(ms5607Temp); Serial.println("C");
  Serial.print("Gas Sensors: ");
  for (int x = 0; x < 8; x++) {
    Serial.print(gasValues[x]); Serial.print(",");
  }
  Serial.println(gasValues[8]);
  Serial.print("SHT11: ");
  Serial.print(shtTemp); Serial.print("C, ");
  Serial.print(shtHumidity); Serial.println("%RH");
  Serial.print("Light: "); Serial.print(lightVal); Serial.println("%");
}

void debugGpsPrint() {
  Serial.print("GPS: ");
  Serial.print(gpsDate); Serial.print(",");
  Serial.print(gpsTime); Serial.print(" ");
  Serial.print(gpsLat); Serial.print(",");
  Serial.print(gpsLng); Serial.print(" ");
  Serial.print("Sats="); Serial.print(gpsSats); Serial.print(" ");
  Serial.print("HDOP="); Serial.print(gpsHdop); Serial.print(" ");
  Serial.print(gpsAlt); Serial.print("m ");
  Serial.print(gpsSpeed); Serial.print("m/s ");
  Serial.print(gpsCourse); Serial.println("deg");
  Serial.print(gpsDistAway); Serial.print("m ");
  Serial.print(gpsCourseTo); Serial.print("deg ");
  Serial.print("("); Serial.print(gpsCourseToText); Serial.println(")");
}

void smsHandler(String smsMessageRaw, bool execCommand, bool smsStartup) {
  String smsRecNumber = "";
  String smsMessage = "";

  int numIndex = smsMessageRaw.indexOf('"') + 3;
  int smsIndex = smsMessageRaw.lastIndexOf('"') + 3;

  for (numIndex; ; numIndex++) {
    char c = smsMessageRaw.charAt(numIndex);
    if (c == '"') break;
    smsRecNumber += c;
  }

  for (smsIndex; ; smsIndex++) {
    char c = smsMessageRaw.charAt(smsIndex);
    if (c == '\n' || c == '\r') break;
    smsMessage += c;
  }

  logDebug("Received SMS message: " + String(smsMessage));

  if (smsStartup == true) {
    if (smsMessage == "Ready") smsReadyReceived = true;
    else startupFailure();
  }

  else if (execCommand == true) {
    int smsCommand = 0;

    if (smsMessage.length() == 1) smsCommand = smsMessage.toInt();
    else; // Send SMS stating invalid command received (to incoming number)

    // Some sort of "if data available, then proceed to switch case"
    switch (smsCommand) {
      // LED
      case 1:
        Serial.print("SMS command #1 issued...");

        for (int x = 0; x < 5; x++) {
          digitalWrite(programStartLED, LOW);
          delay(100);
          digitalWrite(programStartLED, HIGH);
          delay(100);
        }
        break;

      // Location (Google Maps link sent via SMS)
      case 2:
        Serial.print("SMS command #2 issued...");

        Serial1.print("AT+CMGS=\""); Serial1.print(smsTargetNum); Serial1.println("\"");
        delay(100);
        Serial1.print("http://maps.google.com/maps?q=HAB@");
        Serial1.print(gpsLat, 6); Serial1.print(","); Serial1.print(gpsLng, 6);
        Serial1.println("&t=h&z=19&output=html");
        delay(100);
        Serial1.println((char)26);
        delay(100);
        smsFlush();
        smsMarkFlush = true;
        break;

      // Buzzer
      case 3:
        Serial.print("SMS command #3 issued...");

        digitalWrite(buzzerPin, HIGH);
        if (debugMode) delay(1000);
        else delay(30000);
        digitalWrite(buzzerPin, LOW);
        break;
      default:
        Serial.print("INVALID COMMAND: ");
        Serial.print(smsMessage);
        break;
    }

    if (smsCommand == 1) smsCommandText = "LED";
    else if (smsCommand == 3) smsCommandText = "Buzzer";

    logDebug("SMS command issued: " + String(smsCommand));
  }
  else {
    if (debugMode) {
      Serial.print("smsMessage: ");
      Serial.println(smsMessage);
    }
  }
}

void smsMenu() {
  Serial1.print("AT + CMGS = \""); Serial1.print(smsTargetNum); Serial1.println("\"");
  delay(100);
  Serial1.print("SMS Command Menu: ");
  Serial1.print("1-LED, ");
  Serial1.print("2-Location, ");
  Serial1.println("3-Buzzer");
  delay(100);
  Serial1.println((char)26);
  delay(100);
  smsFlush();
  smsMarkFlush = true;
}

void smsSendConfirmation() {
  Serial1.print("AT + CMGS = \""); Serial1.print(smsTargetNum); Serial1.println("\"");
  delay(100);
  Serial1.print(smsCommandText);
  Serial1.println(" activated.");
  delay(100);
  Serial1.println((char)26);
  delay(100);
  smsFlush();
  smsCommandText = "";
  smsMarkFlush = true;
}

void smsFlush() {
  if (Serial1.available()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      delay(10);
    }
  }
}

void startupFailure() {
  while (true) {
    digitalWrite(gpsReadyLED, LOW); digitalWrite(programStartLED, HIGH);
    delay(500);
    digitalWrite(gpsReadyLED, HIGH); digitalWrite(programStartLED, LOW);
    delay(500);
  }
}

void heartbeat() {
  digitalWrite(heartbeatOutputPin, HIGH);
  delay(10);
  digitalWrite(heartbeatOutputPin, LOW);
}