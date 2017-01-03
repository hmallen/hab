/*
   High-Altitude Balloon

  Features:
  - Sensor data acquisition
  - Relay control
  - SD data logging
  - SSB RTTY transmission of position information at 434.250MHz
  - SMS notifications

  Sensors:
  - GPS
  - Gas Sensors
  - Temp/Humidity (SHT11)
  - Light
  - Adafruit 1604 (DOF)
  -- Accelerometer/Gyroscope/Magnetometer
  -- Barometer (Altitude/Temperature) [BMP180]
  - MS5607 Barometer/Altimeter

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
  - Add more EEPROM switches for all capture/phase states
  - ADD TIMEOUT TO MAKE SURE THAT FINAL LANDING COORDINATE ARE SENT
  - ADD NECESSARY BOOLEAN SWITCHES TO RESUME WHERE LEFT AFTER REBOOT
  - ADD PHASE BOOLEAN CHANGES IF PHASE CHANGES OUT OF LOOP
  - MUST FIND FUNCTION TO CONFIRM GPRS POWER TO RESTART IF NECESSARY!!!! ****
  - Confirm that GPS coordinates are sent with highest precision (i.e. 6 floating point decimals)
  - Add check and retry for MS5607 data validity (every so often a bad value appears) ****
  - Add DS1820B data validity check to prevent accidental relay trip
  -- Also add startup check function (i.e. In initSensors())
  - On SMS startup, input current SLP to provide altimeter offset????
  - Handling of gas sensor logging after sensor shut-off!!!!
  - Determine pressure value to trigger peak capture (Reference press vs. alt table & balloon data)
  - TURN ON ROAMING BEFORE LIVE LAUNCH TO ENSURE PRESENCE OF GPRS NETWORK CONNECTION
  - CHANGE GAS SENSOR WARMUP BACK TO NORMAL BEFORE LIVE LAUNCH
  - Add RTTY test TX before SMS confirmation?
  - Add MS5607 altitude????
  - Confirm that boolean argument in smsPower function actually necessary
  - Test if SMS stores in buffer when no signal and sends when reconnected
  - Add check to confirm GPRS is powered on (Need to find suitable function)
  -- Also check if similar function to indicate network connectivity
  - Gather sender number and log incoming/outgoing SMS messages to SD card
  - Consider setting sampling rate based on theoretical ascent rate
  - Change global variables to functions returning pointer arrays

  CONSIDERATIONS:
  - If Python script function timeouts could potentially cause gaps in media acquisition if malfunctioning
  - Setting SD to SPI_FULL_SPEED
  - Inclusion of additional startup SMS output (gas sensor warmup, etc.)
  - Safer to power GPRS before other things to ensure network connectivity?
  - Check if system resets if on external power and serial (computer debugging) is unplugged
  - Gas sensor calibration
  - Possible to use SMS command to reset system or place into "recovery" mode?
  - Create LED flashes to indicate specific startup failure
  - Check if break in DOF logging for other data affects reconstruction
  -- Create program (?Python?) to extrapolate and fill in gaps ["smoothing function"]

  LESSONS LEARNED:
  - I2C device failures (first observed w/ MS5607 CRC4 check fail) likely due to poor jumper/breadboard wiring
  - Debug logging of SMS data currently breaks SMS functions if executed immediately prior
  - All connections within reset circuit must be firmly secured or false resets/none on serial monitor opening occur
  - ??Must allow to pass through first loop before sending SMS command or it will be flushed??
*/

// Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <DHT.h>
#include <EEPROM.h>
#include <MS5xxx.h>
#include <OneWire.h>
#include <SdFat.h>
//#include <SHT1x.h>
#include <SPI.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <util/crc16.h>
#include <Wire.h>

// Definitions
#define GPSHDOPTHRESHOLD 200  // Change to 120 for live conditions????
#define GASSENSORWARMUP 300000  // Gas sensor warm-up time (ms)
//#define GASSENSORWARMUP 30000 // For field stress testing
#define DOFDATAINTERVAL 500 // Update interval (ms) for Adafruit 1604 data
#define AUXDATAINTERVAL 5000 // Update interval (ms) for data other than that from Adafruit 1604
#define GPSDATAINTERVAL 15000 // Update interval (ms) for GPS data updates
#define SMSPOWERDELAY 8000  // Delay between power up and sending of commands to GPRS
#define SERIALTIMEOUT 5000  // Time to wait for incoming GPRS serial data before time-out
#define DOFALTCHANGETHRESHOLD 5.0  // Negative altitude change (m) from altimeters to signify real altitude decrease
#define BAROPRESSCHANGETHRESHOLD 0.5 // Negative pressure change (hPa) from altimeters to signify real altitude decrease
#define GPSCHANGETHRESHOLD 5.0 // Negative altitude change (m) from GPS to signify real altitude decrease
#define HEATERTRIGGERTEMP 0.0  // Temperature (C) below which internal payload heater activated
#define BUZZERACTIVETIME 30000  // Time (ms) that buzzer remains active after triggered by SMS command
#define LAUNCHCAPTURETHRESHOLD 5000.0
#define PEAKCAPTURETHRESHOLD 28.0  // NEED TO CHANGE!!!!
#define LANDINGCAPTURETHRESHOLD 5000.0
#define PHOTODEPLOYTIME 240000

//#define DAYLIGHTSAVINGS

#ifdef DAYLIGHTSAVINGS
const int gpsTimeOffset = -4;
#else
const int gpsTimeOffset = -5;
#endif

// DEBUG
const bool debugMode = false;
const bool debugSmsOff = false;
const bool debugHeaterOff = false;
//const bool debugInputMode = true;
const int debugLED = 13;
const int debugHeaterPin = A1;
const int debugStatePin = A2;
//bool debugState, debugHeaterState;

// Digital Pins
const int chipSelect = SS;
//const int shtData = 2;
//const int shtClock = 3;
const int dhtPin = 2;
const int buzzerRelay = 4;
const int heaterRelay = 5;  // Internal payload heater
const int gasRelay = 6;  // Gas sensors
const int relayPin4 = 7;
const int rttyTxPin = 10;
const int dsTempPin = 11;
const int photoDeployPin = 12;
const int smsPowerPin = 22;
const int gpsPpsPin = 23;
const int programStartPin = 24;
const int programReadyPin = 25;
const int heartbeatOutputPin = 26;
const int accessoryPin = 27;
const int gpsReadyLED = 28; // Multi-color LED round-side input [Green]
const int programStartLED = 29; // Multi-color LED flat-side input [Red]

// Analog Pins
const int lightPin = A0;
//const int gasPins[] = {A7, A8, A9, A10, A11, A12, A13, A14, A15};
const int gasPins[] = {
  A8, A9, A10, A11, A12, A13
};

// Constants
const char dof_log_file[] = "dof_log.txt";
const char aux_log_file[] = "aux_log.txt";
const char gps_log_file[] = "gps_log.txt";
const char debug_log_file[] = "debug_log.txt";
const char smsTargetNum[] = "+12145635266"; // Hunter
const char callsignHeader[] = "$$KG5CKI";

// Global variables
bool gpsTimeSet = false;
bool smsReadyReceived = false;
bool smsFirstFlush = false;
bool smsMarkFlush = false;
char smsCommandText[6];
bool gpsFix = false;
float gpsLaunchLat, gpsLaunchLng;
float gpsLat, gpsLng, gpsAlt, gpsSpeed;
uint32_t gpsDate, gpsTime, gpsSats, gpsCourse;
float gpsDistAway, gpsCourseTo;
//String gpsCourseToText;
int32_t gpsHdop;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;  // Can this be a const float??
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float dofRoll, dofPitch, dofHeading;
float dofPressure, dofTemp, dofAlt, dofAltOffset;
float ms5607Temp, ms5607Press;
//float gasValues[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float gasValues[] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};
//float gasValuesLast[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float gasValuesLast[] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};
//float shtTemp, shtHumidity;
float dhtTemp, dhtHumidity;
float lightVal;
unsigned long buzzerStart;
float dsTemp;

// RPi Camera Triggers
bool takeoffCapture = false;
bool peakCapture = false;
bool landingCapture = false;
bool resetHandler = false;
unsigned long photoDeployStart;
bool selfieRetract = false;

// Old values to process with checkChange() function
float gpsLatLast, gpsLngLast, gpsAltLast, ms5607PressLast, dofAltLast;
int gpsChanges = 0;
int ms5607Changes = 0;
int dofChanges = 0;


// EEPROM Switches
bool setupComplete = false; // EEPROM #0
// takeoffCapture --> EEPROM #1
// peakCapture --> EEPROM #2
bool descentPhase = false;  // EEPROM #3
// landingCapture --> EEPROM #4
bool landingPhase = false;  // EEPROM #5
// resetHandler --> EEPROM #6
// selfieRetract --> EEPROM #7

int loopCount = 1;
unsigned long dofLoopCount = 1;
unsigned long auxLoopCount = 1;
char auxLoopCountChar[6];
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
//SHT1x sht(shtData, shtClock);
DHT dht(dhtPin, DHT22);
TinyGPSPlus gps;
OneWire ds(dsTempPin);

// Data validation variables
//bool dofValid, ms5607Valid, gpsValid, gasValid, shtValid, lightValid, dsValid;
bool dofValid, ms5607Valid, gpsValid, gasValid, dhtValid, lightValid, dsValid;
int ada1604Failures = 0;
int gpsFailures = 0;
int ms5607Failures = 0;
int gasFailures = 0;
//int shtFailures = 0;
int dhtFailures = 0;
int lightFailures = 0;
int dsFailures = 0;

void initSensors() {
  // Adafruit 1604
  if (debugMode) Serial.print(" - DOF...");
  if (!accel.begin()) {
    if (!setupComplete) {
      if (debugMode) Serial.println("failed.");
      startupFailure();
    }
    else if (!debugMode) {
      char debugString[] = "No LSM303 detected.";
      logDebug(debugString);
    }
    else Serial.println("No LSM303 detected.");
  }
  if (!mag.begin()) {
    if (!setupComplete) {
      if (debugMode) Serial.println("failed.");
      startupFailure();
    }
    else if (!debugMode) {
      char debugString[] = "No LSM303 detected.";
      logDebug(debugString);
    }
    else Serial.println("No LSM303 detected.");
  }
  if (!gyro.begin()) {
    if (!setupComplete) {
      if (debugMode) Serial.println("failed.");
      startupFailure();
    }
    else if (!debugMode) {
      char debugString[] = "No L3GD20 detected.";
      logDebug(debugString);
    }
    else Serial.println("No L3GD20 detected.");
  }
  if (!bmp.begin()) {
    if (!setupComplete) {
      if (debugMode) Serial.println("failed.");
      startupFailure();
    }
    else if (!debugMode) {
      char debugString[] = "No BMP180 detected.";
      logDebug(debugString);
    }
    else Serial.println("No BMP180 detected.");
  }
  if (debugMode) {
    Serial.println("success.");
    Serial.print(" - MS5607...");
  }
  if (ms5607.connect() > 0) {
    if (!setupComplete) {
      if (debugMode) Serial.println("failed.");
      startupFailure();
    }
    else if (!debugMode) {
      char debugString[] = "No MS5607 detected.";
      logDebug(debugString);
    }
    else Serial.println("No MS5607 detected.");
  }
  if (debugMode) {
    Serial.println("success.");
    //Serial.print(" - SHT11...");
    Serial.print(" - DHT22...");
  }

  dht.begin();
  delay(2000);
  float t = dht.readTemperature();
  delay(500);
  float h = dht.readHumidity();

  if (isnan(t) || isnan(h)) {
    if (!setupComplete) {
      if (debugMode) Serial.println("failed.");
      //startupFailure();
    }
    else if (!debugMode) {
      char debugString[] = "No DHT22 detected.";
      logDebug(debugString);
    }
  }
  else {
    for (int x = 0; x < 5; ) {
      /*float initShtTempC = sht.readTemperatureC();
        delay(100);
        float initShtTempF = sht.readTemperatureF();
        delay(100);
        float initShtHumidity = sht.readHumidity();
        delay(100);*/

      delay(2000);
      t = dht.readTemperature();
      delay(500);
      h = dht.readHumidity();

      if (!isnan(t) || !isnan(h)) x++;
      else {
        dhtFailures++;
        if (debugMode) Serial.print("DHT read failed...");
      }
    }
  }
  if (debugMode) Serial.println("success.");
}

void initGps() {
  bool gpsPower = false;
  bool gpsPpsVal;
  int gpsPpsPulses;

  if (debugMode) Serial.print("Checking for GPS power...");
  while (!gpsPower) {
    gpsPpsPulses = 0;
    for (unsigned long startTime = millis(); (millis() - startTime) < 1000; ) {
      gpsPpsVal = digitalRead(gpsPpsPin);
      if (gpsPpsVal) gpsPpsPulses++;
      delay(100);
    }
    if (gpsPpsPulses < 5) gpsPower = true;
  }
  if (debugMode) {
    Serial.println("confirmed.");
    Serial.print("Waiting for GPS fix...");
  }
  while (!gpsFix) {
    gpsPpsPulses = 0;
    for (unsigned long startTime = millis(); (millis() - startTime) < 6000; ) {
      gpsPpsVal = digitalRead(gpsPpsPin);
      if (gpsPpsVal) gpsPpsPulses++;
      delay(100);
    }
    if (gpsPpsPulses >= 5) gpsFix = true;
  }
  if (debugMode) {
    Serial.println("attained.");
    Serial.print("Waiting for sufficient HDOP...");
  }

  readGps();

  if (gpsLat == 0.0 || gpsLng == 0.0) startupFailure();
  else {
    while (gpsHdop >= GPSHDOPTHRESHOLD) {
      delay(1000);
      readGps();
      if (debugMode) {
        Serial.print("gpsHdop: ");
        Serial.println(gpsHdop);
      }
    }
    if (debugMode) {
      Serial.println("attained.");
      Serial.print("Waiting for stable launch site coordinates...");
    }
    gpsLaunchLat = gpsLat;
    gpsLaunchLng = gpsLng;
    readGps();
    if (gpsDistAway >= 5.0) {
      while (gpsDistAway >= 5.0) {
        gpsLaunchLat = gpsLat;
        gpsLaunchLng = gpsLng;
        readGps();
        delay(1000);
      }
    }
    if (debugMode) Serial.println("attained.");

    EEPROM.put(10, gpsLaunchLat);
    EEPROM.put(20, gpsLaunchLng);

    if (debugMode) {
      Serial.print("Launch Coordinates: ");
      Serial.print(gpsLaunchLat, 6);
      Serial.print(", ");
      Serial.println(gpsLaunchLng, 6);
      Serial.println();
    }

    digitalWrite(gpsReadyLED, HIGH);
  }
}

void smsPower(bool powerState) {
  // Power on GPRS and set proper modes for operation
  if (powerState) {
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
  smsFlush();

  char smsMessageRaw[128];
  while (!smsReadyReceived) {
    if (!Serial1.available()) {
      while (!Serial1.available()) {
        delay(100);
        digitalWrite(gpsReadyLED, LOW);
        delay(100);
        digitalWrite(gpsReadyLED, HIGH);
      }
    }
    if (Serial1.available()) {
      int x = 0;
      while (Serial1.available()) {
        char c = Serial1.read();
        smsMessageRaw[x] = c;
        x++;
        delay(5);
      }
      smsMessageRaw[x] = '\0';
      smsHandler(smsMessageRaw, false, true);
    }
  }
}

void setup() {
  pinMode(chipSelect, OUTPUT);
  pinMode(gasRelay, OUTPUT);
  digitalWrite(gasRelay, HIGH);  // Gas sensors
  pinMode(heaterRelay, OUTPUT);
  digitalWrite(heaterRelay, LOW); // Payload heater
  pinMode(buzzerRelay, OUTPUT);
  digitalWrite(buzzerRelay, LOW);
  pinMode(relayPin4, OUTPUT);
  digitalWrite(relayPin4, LOW);
  pinMode(rttyTxPin, OUTPUT);
  pinMode(smsPowerPin, OUTPUT);
  digitalWrite(smsPowerPin, LOW);
  pinMode(gpsReadyLED, OUTPUT);
  digitalWrite(gpsReadyLED, LOW);
  pinMode(programStartLED, OUTPUT);
  digitalWrite(programStartLED, LOW);
  pinMode(programReadyPin, OUTPUT);
  digitalWrite(programReadyPin, LOW);
  pinMode(heartbeatOutputPin, OUTPUT);
  digitalWrite(heartbeatOutputPin, LOW);
  pinMode(buzzerRelay, OUTPUT);
  digitalWrite(buzzerRelay, LOW);
  pinMode(debugLED, OUTPUT);
  digitalWrite(debugLED, HIGH);
  pinMode(photoDeployPin, OUTPUT);
  digitalWrite(photoDeployPin, HIGH);

  pinMode(debugStatePin, INPUT_PULLUP);
  pinMode(debugHeaterPin, INPUT_PULLUP);
  pinMode(gpsPpsPin, INPUT_PULLUP);
  pinMode(programStartPin, INPUT_PULLUP);

  Serial.begin(115200); // Python serial communication & Debug output
  Serial1.begin(19200); // GPRS communication
  Serial2.begin(9600);  // GPS communication

  if (debugMode) {
    Serial.println();
    Serial.println(F("---------------"));
    Serial.println(F("---- SETUP ----"));
    Serial.println(F("---------------"));
    Serial.println();
  }

  if (digitalRead(programStartPin) == LOW) {
    if (debugMode) Serial.print("Clearing EEPROM...");
    for (int x = 0; x < 8; x++) {
      EEPROM.update(x, 0);
    }
    for (int x = 10; x < 14; x++) {
      EEPROM.update(x, 0);
    }
    for (int x = 20; x < 24; x++) {
      EEPROM.update(x, 0);
    }
    if (debugMode) {
      Serial.println("complete.");
      Serial.println();
    }
  }

  if (debugMode) Serial.println("Initializing sensors:");
  initSensors();
  if (debugMode) {
    Serial.println("Complete.");
    Serial.println();
    Serial.print("Initializing SD card...");
  }
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    if (!setupComplete) startupFailure();
    else {
      if (debugMode) {
        for (int x = 0; x < 10; x++) {
          digitalWrite(programStartLED, HIGH);
          delay(250);
          digitalWrite(programStartLED, LOW);
          delay(250);
        }
      }
    }
  }
  if (debugMode) {
    Serial.println("complete.");
    Serial.println();
  }

  setupComplete = EEPROM.read(0);
  if (setupComplete) {
    digitalWrite(gpsReadyLED, HIGH);

    // EEPROM #1 --> Takeoff capture
    // EEPROM #2 --> Peak capture
    // EEPROM #3 --> Descent phase
    // EEPROM #4 --> Landing capture
    // EEPROM #5 --> Landing phase
    // EEPROM #6 --> Reset handler
    // EEPROM #7 --> Selfie retract

    takeoffCapture = EEPROM.read(1);
    peakCapture = EEPROM.read(2);
    descentPhase = EEPROM.read(3);
    landingCapture = EEPROM.read(4);
    landingPhase = EEPROM.read(5);
    resetHandler = EEPROM.read(6);
    selfieRetract = EEPROM.read(7);

    if (debugMode) {
      Serial.println("EEPROM Values:");
      Serial.print("takeoffCapture: ");
      Serial.println(takeoffCapture);
      Serial.print("peakCapture: ");
      Serial.println(peakCapture);
      Serial.print("descentPhase: ");
      Serial.println(descentPhase);
      Serial.print("landingCapture: ");
      Serial.println(landingCapture);
      Serial.print("landingPhase: ");
      Serial.println(landingPhase);
      Serial.print("resetHandler: ");
      Serial.println(resetHandler);
      Serial.print("selfieRetract: ");
      Serial.println(selfieRetract);
    }

    EEPROM.get(10, gpsLaunchLat);
    EEPROM.get(20, gpsLaunchLng);

    if (descentPhase || landingPhase) digitalWrite(gasRelay, LOW);

    smsFlush();

    if (debugMode) {
      Serial.println("System rebooted by watchdog timer or manual reset.");
      Serial.println();
    }
    else {
      char debugString[] = "System rebooted by watchdog timer or manual reset.";
      logDebug(debugString);
    }
  }
  else {
    initGps();

    if (debugMode) Serial.print("Powering GPRS...");
    smsPower(true);
    if (debugMode) Serial.println("complete.");

    delay(SMSPOWERDELAY);

    for (int x = 0; x < 2; x++) {
      digitalWrite(programStartLED, HIGH);
      delay(100);
      digitalWrite(programStartLED, LOW);
      delay(100);
    }

    if (!debugSmsOff) {
      if (debugMode) {
        Serial.print("Sending SMS confirmation request...");
        Serial.flush();
      }
      Serial1.print("AT + CMGS = \"");
      Serial1.print(smsTargetNum);
      Serial1.println("\"");
      delay(100);
      Serial1.println("Reply with \"Ready\" to continue.");
      delay(100);
      Serial1.println((char)26);
      delay(100);
      smsFlush();

      if (!Serial1.available()) {
        while (!Serial1.available()) {
          delay(10);
        }
      }
      smsFlush();

      if (debugMode) {
        Serial.println("complete.");
        Serial.print("Waiting for SMS ready message...");
      }
      smsConfirmReady();
      if (debugMode) Serial.println("received.");
    }
    if (debugMode) Serial.println();

    while (true) {
      if (debugMode) Serial.print("Baselining launch site altitude for comparison...");
      float altTotal = 0;
      for (int x = 0; x < 5; x++) {
        readAda1604();
        altTotal += dofAlt;
        delay(1000);
      }
      dofAltOffset = altTotal / 5;
      readAda1604();
      if (abs(dofAlt - dofAltOffset) < 5.0) break;
    }
    if (debugMode) {
      Serial.println("complete.");
      Serial.println();
    }

    //smsPower(false);

    for (int x = 0; x < 2; x++) {
      digitalWrite(programStartLED, HIGH);
      delay(100);
      digitalWrite(programStartLED, LOW);
      delay(100);
    }

    if (debugMode) Serial.print("Warming-up gas sensors...");
    else {
      for (unsigned long startTime = millis(); (millis() - startTime) < GASSENSORWARMUP; ) {
        for (int x = 0; x < 2; x++) {
          delay(100);
          digitalWrite(programStartLED, HIGH);
          delay(100);
          digitalWrite(programStartLED, LOW);
        }
        delay(1000);
      }
    }
    if (debugMode) {
      Serial.println("complete.");
      Serial.println();
    }

    if (digitalRead(programStartPin) == LOW) {
      if (debugMode) Serial.print("Toggle start switch to continue...");
      while (digitalRead(programStartPin) == LOW) {
        digitalWrite(programStartLED, HIGH);
        delay(500);
        digitalWrite(programStartLED, LOW);
        delay(500);
      }
    }

    Serial.println("$0");
    //if (!debugMode) {
    if (!Serial.available()) {
      while (!Serial.available()) {
        ;
      }
    }
    while (true) {
      if (Serial.available()) {
        char cameraInput[6];
        int x = 0;
        while (Serial.available()) {
          char c = Serial.read();
          cameraInput[x] = c;
          x++;
          delay(5);
        }
        cameraInput[x] = '\0';
        if (cameraInput[0] == '$' && cameraInput[1] == '0') break;
      }
    }
  }

  if (debugMode) {
    Serial.println("starting program.");
    Serial.println();
    Serial.println(F("-----------------"));
    Serial.println(F("---- PROGRAM ----"));
    Serial.println(F("-----------------"));
    Serial.println();
  }

  EEPROM.update(0, 1);
  digitalWrite(programReadyPin, HIGH);
  digitalWrite(programStartLED, LOW);
  digitalWrite(gpsReadyLED, LOW);
}

void loop() {
  if (debugMode) {
    Serial.print("Loop #: ");
    Serial.println(loopCount);
    Serial.println();
  }
  unsigned long loopStart = millis();
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 10; y++) {
      unsigned long startTimeDof = millis();
      dofValid = readAda1604();
      if (!dofValid) {
        ada1604Failures++;
        if (debugMode) {
          Serial.print("Failed to read DOF. Count=");
          Serial.println(ada1604Failures);
        }
        else {
          char debugChar[64];
          char dofFailuresChar[6];
          char debugPrefix[] = "Failed to read DOF. Count=";
          sprintf(debugChar, debugPrefix);
          sprintf(dofFailuresChar, "%i", ada1604Failures);
          strcat(debugChar, dofFailuresChar);
          logDebug(debugChar);
        }
      }
      if (debugMode) {
        Serial.print("DOF Loop #: ");
        Serial.println(dofLoopCount);
        debugDofPrint();
        Serial.print("Time(DOF): ");
        Serial.println((millis() - startTimeDof));
      }
      else {
        char dataType[] = "dof";
        logData(dataType);
      }
      delay(DOFDATAINTERVAL);
      dofLoopCount++;

      heartbeat();
    }
    if (debugMode) {
      Serial.println();
      Serial.print("Time(Aux-START): ");
      Serial.println((millis() - loopStart));
    }

    ms5607Valid = readMS5607();
    if (!ms5607Valid) {
      ms5607Failures++;
      if (debugMode) {
        Serial.print("Failed to read MS5607. Count=");
        Serial.println(ms5607Failures);
      }
      else {
        char debugChar[64];
        char ms5607FailuresChar[6];
        char debugPrefix[] = "Failed to read MS5607. Count=";
        sprintf(debugChar, debugPrefix);
        sprintf(ms5607FailuresChar, "%i", ms5607Failures);
        strcat(debugChar, ms5607FailuresChar);
        logDebug(debugChar);
      }
      for (int x = 0; x < 5; x++) {
        if (readMS5607()) break;
        else {
          ms5607Failures++;
          if (debugMode) {
            Serial.print("Failed to read MS5607. Count=");
            Serial.println(ms5607Failures);
          }
          else {
            char debugChar[64];
            char ms5607FailuresChar[6];
            char debugPrefix[] = "Failed to read MS5607. Count=";
            sprintf(debugChar, debugPrefix);
            sprintf(ms5607FailuresChar, "%i", ms5607Failures);
            strcat(debugChar, ms5607FailuresChar);
            logDebug(debugChar);
          }
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
          Serial.print("Failed to read gas sensors. Count=");
          Serial.println(gasFailures);
        }
        else {
          char debugChar[64];
          char gasFailuresChar[6];
          char debugPrefix[] = "Failed to read gas sensors. Count=";
          sprintf(debugChar, debugPrefix);
          sprintf(gasFailuresChar, "%i", gasFailures);
          strcat(debugChar, gasFailuresChar);
          logDebug(debugChar);
        }
      }
    }

    dhtValid = readDht();
    if (!dhtValid) {
      dhtFailures++;
      if (debugMode) {
        Serial.print("Failed to read DHT11. Count=");
        Serial.println(dhtFailures);
      }
      else {
        char debugChar[64];
        char dhtFailuresChar[6];
        char debugPrefix[] = "Failed to read DHT11. Count=";
        sprintf(debugChar, debugPrefix);
        sprintf(dhtFailuresChar, "%i", dhtFailures);
        strcat(debugChar, dhtFailuresChar);
        logDebug(debugChar);
      }
    }

    lightValid = readLight();
    if (!lightValid) {
      lightFailures++;
      if (debugMode) {
        Serial.print("Failed to read light sensor. Count=");
        Serial.println(lightFailures);
      }
      else {
        char debugChar[64];
        char lightFailuresChar[6];
        char debugPrefix[] = "Failed to read light sensor. Count=";
        sprintf(debugChar, debugPrefix);
        sprintf(lightFailuresChar, "%i", lightFailures);
        strcat(debugChar, lightFailuresChar);
        logDebug(debugChar);
      }
    }

    dsValid = readDs();
    if (!dsValid) {
      dsFailures++;
      if (debugMode) {
        Serial.print("Failed to read DS18B20. Count=");
        Serial.println(dsFailures);
      }
      else {
        char debugChar[64];
        char dsFailuresChar[6];
        char debugPrefix[] = "Failed to read DS18B20. Count=";
        sprintf(debugChar, debugPrefix);
        sprintf(dsFailuresChar, "%i", dsFailures);
        strcat(debugChar, dsFailuresChar);
        logDebug(debugChar);
      }
    }

    if (debugMode) {
      Serial.print("Aux Loop #: ");
      Serial.println(auxLoopCount);
    }

    sprintf(auxLoopCountChar, "%i", auxLoopCount);
    auxLoopCount++;

    if (debugMode) {
      debugAuxPrint();
      Serial.print("Time(Aux-END): ");
      Serial.println((millis() - loopStart));
      Serial.println();
    }
    else {
      char dataType[] = "aux";
      logData(dataType);
    }

    if (digitalRead(buzzerRelay) == HIGH && (millis() - buzzerStart) >= BUZZERACTIVETIME) digitalWrite(buzzerRelay, LOW);

    if (debugMode) Serial.print("Sending data via RTTY...");
    rttyProcessTx();
    if (debugMode) {
      Serial.println("complete.");
      Serial.println();
    }

    heartbeat();
  }

  gpsValid = readGps();
  if (!gpsValid) {
    gpsFailures++;
    if (debugMode) {
      Serial.print("Failed to read GPS. Count=");
      Serial.println(gpsFailures);
    }
    else {
      char debugChar[64];
      char gpsFailuresChar[6];
      char debugPrefix[] = "Failed to read GPS. Count=";
      sprintf(debugChar, debugPrefix);
      sprintf(gpsFailuresChar, "%i", gpsFailures);
      strcat(debugChar, gpsFailuresChar);
      logDebug(debugChar);
    }
    gpsLat = gpsLatLast;
    gpsLng = gpsLngLast;
  }
  if (debugMode) {
    Serial.print("GPS Loop #: ");
    Serial.println(gpsLoopCount);
  }
  gpsLoopCount++;

  if (debugMode) {
    debugGpsPrint();
    Serial.print("Time(GPS): ");
    Serial.println(millis() - loopStart);
    Serial.println();
  }
  else {
    char dataType[] = "gps";
    logData(dataType);
  }

  if (smsMarkFlush) {
    if (debugMode) Serial.print("Flushing GPRS buffer...");
    smsFlush();
    if (debugMode) {
      Serial.println("complete.");
      Serial.println();
    }
    smsMarkFlush = false;
  }

  if (Serial1.available()) {
    if (debugMode) Serial.println("Incoming GPRS serial data.");
    char smsMessageRaw[128];
    int x = 0;
    while (Serial1.available()) {
      char c = Serial1.read();
      smsMessageRaw[x] = c;
      x++;
      delay(5);
    }
    smsMessageRaw[x] = '\0';
    if (debugMode) Serial.print("Processing GPRS data...");
    smsHandler(smsMessageRaw, true, false);
    if (debugMode) {
      Serial.println("complete.");
      Serial.println();
    }
  }
  /*if (Serial1.available()) {
    if (debugMode) Serial.println("Incoming GPRS serial data.");
    char smsMessageRaw[128];
    int x = 0;
    while (Serial1.available()) {
      char c = Serial1.read();
      smsMessageRaw[x] = c;
      x++;
      delay(5);
    }
    if (debugMode) Serial.print("Processing GPRS data...");
    smsHandler(smsMessageRaw, true, false);
    if (debugMode) {
      Serial.println("complete.");
      Serial.println();
    }
    }*/

  // if (smsCommandText[0] != '\0') {
  /*if (!isAlphaNumeric(smsCommandText[0])) {
    Serial.print("smsCommandText: ");
      Serial.println(smsCommandText);
      Serial.println("PROGRAM HALTED.");
      while (true) {
      ;
      }
      smsSendConfirmation();
    smsFlush();
    }*/

  if (isAlphaNumeric(smsCommandText[0])) {
    smsSendConfirmation();
    smsFlush();
  }

  if (loopCount == 1 && !descentPhase && !landingPhase) {
    ms5607PressLast = ms5607Press;
    if (!debugSmsOff) {
      if (debugMode) {
        Serial.print("Sending SMS command menu...");
      }
      smsMenu();
      if (debugMode) {
        Serial.println("complete.");
        Serial.println();
      }
    }
  }
  else {
    checkChange();
    if (debugMode) {
      Serial.print("GPS Changes: ");
      Serial.println(gpsChanges);
      Serial.print("DOF Changes: ");
      Serial.println(dofChanges);
      Serial.print("MS5607 Changes: ");
      Serial.println(ms5607Changes);
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

    if (!gpsTimeSet) {
      unsigned long age;
      uint8_t Month = gps.date.month();
      uint8_t Day = gps.date.day();
      uint8_t Hour = gps.time.hour();
      uint8_t Minute = gps.time.minute();
      uint8_t Second = gps.time.second();

      uint16_t yearRaw = gps.date.year();
      char yearCharRaw[6];
      char yearChar[3];

      sprintf(yearCharRaw, "%i", yearRaw);

      for (int x = 2; x < 4; x++) {
        char c = yearCharRaw[x];
        yearChar[x - 2] = c;
      }
      yearChar[2] = '\0';
      uint16_t Year = atoi(yearChar);

      setTime(Hour, Minute, Second, Day, Month, Year);
      adjustTime(gpsTimeOffset * SECS_PER_HOUR);
      gpsTimeSet = true;
    }

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

    //gpsCourseToText = String(gps.cardinal(gpsCourseTo));

    return true;
  }
  else return false;
}

bool readMS5607() {
  for (int x = 0; x < 5; x++) {
    ms5607.ReadProm();
    ms5607.Readout();

    ms5607Temp = ms5607.GetTemp() / 100.0;
    ms5607Press = ms5607.GetPres() / 100.0;

    if ((-100.0 <= ms5607Temp <= 100.0) && (0.0 <= ms5607Press <= 1200.0)) return true;
    else continue;

    delay(500);
  }
  return false;
}

bool readGas() {
  int gasPinLength = sizeof(gasPins) / 2;
  if (!descentPhase) {
    for (int x = 0; x < gasPinLength; x++) {
      gasValuesLast[x] = gasValues[x];
    }

    for (int x = 0; x < gasPinLength; x++) {
      gasValues[x] = (float)map(analogRead(gasPins[x]), 0, 1023, 0, 1000) / 10.0;
      delay(10);
    }
  }
  else {
    for (int x = 0; x < gasPinLength; x++) {
      gasValues[x] = 0.0;
    }
  }

  if (gasValues != gasValuesLast) return true;
  else return false;
}

/*bool readSht() {
  shtTemp = sht.readTemperatureC();
  shtHumidity = sht.readHumidity();

  if (-100.0 <= shtTemp <= 100.0 && 0.0 <= shtHumidity <= 100.0) return true;
  else return false;
  }*/

bool readDht() {
  dhtTemp = dht.readTemperature();
  delay(250);
  dhtHumidity = dht.readHumidity();

  if (!isnan(dhtTemp) && !isnan(dhtHumidity)) return true;
  else return false;
}

bool readLight() {
  lightVal = (float)map((1023 - analogRead(lightPin)), 0, 1023, 0, 1000) / 10.0;
  if (0.0 <= lightVal <= 100.0) return true;
  else return false;
}

bool readDs() {
  bool dsReadCheck = false;

  for (int x = 0; x < 3; x++) {
    byte data[12];
    byte addr[8];

    if (!ds.search(addr)) {
      ds.reset_search();
      delay(250);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
      //char debugString[32];
      //sprintf(debugString, "DS18B20 CRC not valid.");
      char debugString[] = "DS18B20 CRC not valid.";
      logDebug(debugString);
      dsTemp = 0.0;
      continue;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);  // start conversion, with parasite power on at the end

    delay(1000);  // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    for (byte i = 0; i < 9; i++) {  // we need 9 bytes
      data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    byte type_s;
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    }
    else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    dsTemp = (float)raw / 16.0;
    dsReadCheck = true;
    break;
  }
  if (dsReadCheck) return true;
  else return false;
}

void checkChange() {
  static bool heaterStatus = digitalRead(heaterRelay);
  float gpsAltChange = gpsAltLast - gpsAlt;
  float dofAltChange = dofAltLast - dofAlt;
  float ms5607PressChange = ms5607Press - ms5607PressLast;

  bool debugState = digitalRead(debugStatePin);
  bool debugHeaterState = digitalRead(debugHeaterPin);

  // Heater functions
  if (!debugHeaterOff) {
    if (!landingPhase) {
      if (dsValid && dsTemp < HEATERTRIGGERTEMP && !heaterStatus || !debugHeaterState && !heaterStatus) {
        if (!debugHeaterState) debugBlink();
        digitalWrite(heaterRelay, HIGH);
        heaterStatus = true;
        char debugChar[64];
        char debugPrefix[] = "Heater activated @ ";
        char dofTempChar[6];
        dtostrf(dofTemp, 2, 2, dofTempChar);
        sprintf(debugChar, debugPrefix);
        strcat(debugChar, dofTempChar);
        logDebug(debugChar);
      }

      else if (dsValid && dsTemp >= HEATERTRIGGERTEMP && heaterStatus || !debugHeaterState && heaterStatus) {
        if (!debugHeaterState) debugBlink();
        digitalWrite(heaterRelay, LOW);
        heaterStatus = false;
        char debugChar[64];
        char debugPrefix[] = "Heater inactivated @ ";
        char dofTempChar[6];
        dtostrf(dofTemp, 2, 2, dofTempChar);
        sprintf(debugChar, debugPrefix);
        strcat(debugChar, dofTempChar);
        logDebug(debugChar);
      }
    }
  }

  else if (landingPhase && heaterStatus) digitalWrite(heaterRelay, LOW);

  // TAKEOFF AND ASCENT
  if (!descentPhase) {
    if (gpsAltChange <= 0.0) gpsChanges = 0;
    else if (gpsAltChange > GPSCHANGETHRESHOLD) gpsChanges++;

    if (dofAltChange <= 0.0) dofChanges = 0;
    else if (dofAltChange > DOFALTCHANGETHRESHOLD) {
      dofChanges++;
      if (debugMode) {
        Serial.print(dofAlt);
        Serial.print(" - ");
        Serial.print(dofAltLast);
        Serial.print(" = ");
        Serial.println(dofAltChange);
      }
    }

    if (ms5607PressChange <= 0.0) ms5607Changes = 0;
    else if (ms5607PressChange > BAROPRESSCHANGETHRESHOLD) {
      ms5607Changes++;
      if (debugMode) {
        Serial.print(ms5607Press);
        Serial.print(" - ");
        Serial.print(ms5607PressLast);
        Serial.print(" = ");
        Serial.println(ms5607PressChange);
      }
    }

    // LAUNCH CAPTURE ACTIVATED AUTOMATICALLY
    if (!takeoffCapture) {
      // START LAUNCH CAP --> resetHandler = true ["2"]
      Serial.println("$1");
      digitalWrite(gpsReadyLED, HIGH);
      takeoffCapture = true;
      EEPROM.update(1, 1);
      resetHandler = true;
      EEPROM.update(6, 1);
    }

    else {
      // LAUNCH CAPTURE ENDS WHEN OVER THRESHOLD ALTITUDE
      if (resetHandler && !peakCapture) {
        // WAIT FOR TRIGGER (ABOVE THRESHOLD ALTITUDE OR DEBUG) --> resetHandler = false
        //// END LAUNCH CAP ["3"]
        if ((dofAlt - dofAltOffset) > LAUNCHCAPTURETHRESHOLD || !debugState) {
          if (!debugState) debugBlink();
          Serial.println("$0");
          digitalWrite(gpsReadyLED, LOW);
          resetHandler = false;
          EEPROM.update(6, 0);
        }
      }

      // PEAK CAPTURE BEGINS WHEN OVER THRESHOLD ALTITUDE
      else if (!resetHandler && !peakCapture) {
        // WAIT FOR TRIGGER (BELOW THRESHOLD PRESSURE) --> peakCapture = true
        // DEPLOY SELFIE SERVO
        // WAIT FOR SERVO TIMEOUT [Should this be outside of loops to ensure that it's eventually called????]
        // RETRACT SELFIE SERVO
        //// resetHandler = true ["4"]
        if (dofPressure < PEAKCAPTURETHRESHOLD || !debugState) {
          if (!debugState) debugBlink();
          Serial.println("$2");
          digitalWrite(programStartLED, HIGH);
          digitalWrite(photoDeployPin, LOW);  // DEPLOYMENT OF SPACE SELFIE!!!!
          selfieRetract = false;
          EEPROM.update(7, 0);
          photoDeployStart = millis();
          peakCapture = true;
          EEPROM.update(2, 1);
          resetHandler = true;
          EEPROM.update(6, 1);
          if (debugMode) {
            Serial.println();
            Serial.println("Peak capture triggered.");
            Serial.println();
          }
        }
      }

      // PEAK CAPTURE ENDS WHEN DESCENT TRIGGERS
      else if (resetHandler && peakCapture) {
        if (!selfieRetract && (millis() - photoDeployStart) > PHOTODEPLOYTIME) {
          digitalWrite(photoDeployPin, HIGH);
          selfieRetract = true;
          EEPROM.update(7, 1);
        }
        // WAIT FOR DESCENT --> descentPhase = true --> resetHandler = false ["5"]
        // ZERO ALL CHANGE COUNTERS
        if (gpsChanges >= 10) descentPhase = true;
        else if (dofChanges >= 10) descentPhase = true;
        else if (gpsChanges >= 5 && dofChanges >= 5) descentPhase = true;
        else if (gpsChanges >= 3 && dofChanges >= 3 && ms5607Changes >= 3) descentPhase = true;
        else if (!debugState) {
          debugBlink();
          descentPhase = true;
        }

        if (descentPhase) {
          Serial.println("$3");
          digitalWrite(programStartLED, LOW);
          EEPROM.write(3, 1);
          if (debugMode) {
            Serial.println();
            Serial.println("Descent phase triggered.");
            Serial.println();
          }
          resetHandler = false;
          EEPROM.update(6, 0);
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

      // IF NO LOGIC MATCHED WITH PROGRAM (PROGRAM FAILURE)
      else if (debugMode) Serial.println("INVALID PROGRAMMING LOGIC");
    }
  }

  // DESCENT PHASE
  else if (!landingPhase) {
    // IN CASE SELFIE SERVO DIDN'T RETRACT
    if (!selfieRetract && (millis() - photoDeployStart) > PHOTODEPLOYTIME) {
      digitalWrite(photoDeployPin, HIGH);
      selfieRetract = true;
      EEPROM.update(7, 1);
    }

    // LANDING CAPTURE BEGINS WHEN BELOW THRESHOLD ALTITUDE
    if (!resetHandler && !landingCapture) {
      // WAIT FOR ALTITUDE BELOW THRESHOLD
      //// BEGIN LANDING CAP --> landingCapture = true --> resetHandler = true
      if ((dofAlt - dofAltOffset) < LANDINGCAPTURETHRESHOLD || !debugState) {
        if (!debugState) debugBlink();
        Serial.println("$3");
        digitalWrite(gpsReadyLED, HIGH);
        landingCapture = true;
        EEPROM.update(4, 1);
        resetHandler = true;
        EEPROM.update(6, 1);
      }
    }

    // LANDING PHASE BEGINS WHEN ALTITUDE STOPS CHANGING
    else if (resetHandler && landingCapture) {
      if (gpsAltChange > GPSCHANGETHRESHOLD) gpsChanges = 0;
      else if (abs(gpsAltChange) <= GPSCHANGETHRESHOLD) gpsChanges++;
      if (dofAltChange > DOFALTCHANGETHRESHOLD) dofChanges = 0;
      else if (abs(dofAltChange) <= DOFALTCHANGETHRESHOLD) dofChanges++;
      if (ms5607PressChange > BAROPRESSCHANGETHRESHOLD) ms5607Changes = 0;
      else if (abs(ms5607PressChange) <= BAROPRESSCHANGETHRESHOLD) ms5607Changes++;

      // WAIT FOR ALTITUDE TO STABILIZE
      //// END LANDING CAP --> landingPhase = true
      if (gpsChanges >= 10) landingPhase = true;
      else if (dofChanges >= 10) landingPhase = true;
      else if (gpsChanges >= 5 && dofChanges >= 5) landingPhase = true;
      else if (gpsChanges >= 3 && dofChanges >= 3 && ms5607Changes >= 3) landingPhase = true;
      else if (!debugState) {
        debugBlink();
        landingPhase = true;
      }
    }

    else if (debugMode) Serial.println("INVALID PROGRAMMING LOGIC");
  }

  // LANDING PHASE
  else if (landingPhase && resetHandler) {
    Serial.println("$4");
    digitalWrite(gpsReadyLED, LOW);
    EEPROM.update(5, 1);
    resetHandler = false;
    EEPROM.update(6, 0);

    if (debugMode) {
      Serial.println();
      Serial.print("Landing phase triggered. Sending location via SMS...");
      Serial.println();
    }

    Serial1.print("AT+CMGS=\"");
    Serial1.print(smsTargetNum);
    Serial1.println("\"");
    delay(100);
    Serial1.print("LANDING DETECTED ");
    Serial1.print(gpsDistAway);
    Serial1.print("m ");
    Serial1.print(gpsCourseTo);
    //Serial1.print(gpsCourseToText);
    Serial1.print("deg from launch site. ");
    Serial1.print("http://maps.google.com/maps?q=HAB@");
    Serial1.print(gpsLat, 6);
    Serial1.print(",");
    Serial1.print(gpsLng, 6);
    Serial1.println("&t=h&z=19&output=html");
    delay(100);
    Serial1.println((char)26);
    delay(100);
    smsFlush();
    smsMarkFlush = true;
    if (debugMode) {
      Serial.print("complete.");
      Serial.println();
    }
  }

  else {
    // STUFF WHILE AT REST IN LANDING PHASE
  }

  gpsAltLast = gpsAlt;
  dofAltLast = dofAlt;
  ms5607PressLast = ms5607Press;
}

void logData(char *logType) {
  // GET RID OF SD.ERRORHALT() TO PREVENT PROGRAM FROM STALLING
  if (logType == "dof") {
    if (!logFile.open(dof_log_file, O_RDWR | O_CREAT | O_AT_END)) {
      if (debugMode) sd.errorHalt("Opening debug log for write failed.");
    }

    time_t t = now();
    logFile.print(loopCount);
    logFile.print(",");
    logFile.print(month(t));
    logFile.print(day(t));
    logFile.print(year(t));
    logFile.print("-");
    logFile.print(hour(t));
    logFile.print(minute(t));
    logFile.print(second(t));
    logFile.print(",");
    logFile.print(dofValid);
    logFile.print(",");
    logFile.print(accelX);
    logFile.print(",");
    logFile.print(accelY);
    logFile.print(",");
    logFile.print(accelZ);
    logFile.print(",");
    logFile.print(gyroX);
    logFile.print(",");
    logFile.print(gyroY);
    logFile.print(",");
    logFile.print(gyroZ);
    logFile.print(",");
    logFile.print(magX);
    logFile.print(",");
    logFile.print(magY);
    logFile.print(",");
    logFile.print(magZ);
    logFile.print(",");
    logFile.print(dofRoll);
    logFile.print(",");
    logFile.print(dofPitch);
    logFile.print(",");
    logFile.print(dofHeading);
    logFile.print(",");
    logFile.print(dofPressure);
    logFile.print(",");
    logFile.print(dofTemp);
    logFile.print(",");
    logFile.println(dofAlt);
  }

  else if (logType == "aux") {
    if (!logFile.open(aux_log_file, O_RDWR | O_CREAT | O_AT_END)) {
      if (debugMode) sd.errorHalt("Opening aux log for write failed.");
    }

    time_t t = now();
    logFile.print(loopCount);
    logFile.print(",");
    logFile.print(month(t));
    logFile.print(day(t));
    logFile.print(year(t));
    logFile.print("-");
    logFile.print(hour(t));
    logFile.print(minute(t));
    logFile.print(second(t));
    logFile.print(",");
    logFile.print(ms5607Valid);
    logFile.print(",");
    logFile.print(ms5607Press);
    logFile.print(",");
    logFile.print(ms5607Temp);
    logFile.print(",");
    for (int x = 0; x < 6; x++) {
      logFile.print(gasValues[x]);
      logFile.print(",");
    }
    //logFile.print(shtValid);
    logFile.print(dhtValid);
    logFile.print(",");
    //logFile.print(shtTemp);
    logFile.print(dhtTemp);
    logFile.print(",");
    //logFile.print(shtHumidity);
    logFile.print(dhtHumidity);
    logFile.print(",");
    logFile.print(lightVal);
    logFile.print(",");
    logFile.println(dsTemp);
  }
  else if (logType == "gps") {
    if (!logFile.open(gps_log_file, O_RDWR | O_CREAT | O_AT_END)) {
      if (debugMode) sd.errorHalt("Opening GPS log for write failed.");
    }

    time_t t = now();
    logFile.print(loopCount);
    logFile.print(",");
    logFile.print(month(t));
    logFile.print(day(t));
    logFile.print(year(t));
    logFile.print("-");
    logFile.print(hour(t));
    logFile.print(minute(t));
    logFile.print(second(t));
    logFile.print(",");
    logFile.print(gpsValid);
    logFile.print(",");
    logFile.print(gpsDate);
    logFile.print(",");
    logFile.print(gpsTime);
    logFile.print(",");
    logFile.print(gpsLat);
    logFile.print(",");
    logFile.print(gpsLng);
    logFile.print(",");
    logFile.print(gpsSats);
    logFile.print(",");
    logFile.print(gpsHdop);
    logFile.print(",");
    logFile.print(gpsAlt);
    logFile.print(",");
    logFile.print(gpsSpeed);
    logFile.print(",");
    logFile.print(gpsCourse);
    logFile.print(",");
    logFile.print(gpsDistAway);
    logFile.print(",");
    logFile.println(gpsCourseTo);
    //logFile.print(",");
    //logFile.println(gpsCourseToText);
  }
  else {
    if (debugMode) Serial.println("Unrecognized log type. Failed to write to SD.");
    else {
      char debugString[] = "Unrecognized log type. Failed to write to SD.";
      logDebug(debugString);
    }
  }

  logFile.flush();
  logFile.close();
}

void logDebug(char *dataString) {
  // GET RID OF SD.ERRORHALT() TO PREVENT PROGRAM FROM STALLING
  if (!logFile.open(debug_log_file, O_RDWR | O_CREAT | O_AT_END)) {
    if (debugMode) sd.errorHalt("Opening debug log for write failed.");
  }

  time_t t = now();
  logFile.print(loopCount);
  logFile.print(",");
  logFile.print(month(t));
  logFile.print(day(t));
  logFile.print(year(t));
  logFile.print("-");
  logFile.print(hour(t));
  logFile.print(minute(t));
  logFile.print(second(t));
  logFile.print(",");
  logFile.println(dataString);
  logFile.flush();
  logFile.close();
}

void debugDofPrint() {
  Serial.print(dofRoll);
  Serial.print("(Roll), ");
  Serial.print(dofPitch);
  Serial.print("(Pitch), ");
  Serial.print(dofHeading);
  Serial.print("(Heading) / ");
  Serial.print(dofPressure);
  Serial.print("hPa, ");
  Serial.print(dofTemp);
  Serial.print("C, ");
  Serial.print(dofAlt);
  Serial.println("m");
  Serial.print("DOF Accel: ");
  Serial.print(accelX);
  Serial.print(",");
  Serial.print(accelY);
  Serial.print(",");
  Serial.println(accelZ);
  Serial.print("DOF Gyro: ");
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.println(gyroZ);
  Serial.print("DOF Mag: ");
  Serial.print(magX);
  Serial.print(",");
  Serial.print(magY);
  Serial.print(",");
  Serial.println(magZ);
}

void debugAuxPrint() {
  Serial.print("MS5607: ");
  Serial.print(ms5607Press);
  Serial.print("hPa, ");
  Serial.print(ms5607Temp);
  Serial.println("C");
  Serial.print("Gas Sensors: ");
  for (int x = 0; x < 5; x++) {
    Serial.print(gasValues[x]);
    Serial.print(",");
  }
  Serial.println(gasValues[5]);
  Serial.print("DHT22: ");
  Serial.print(dhtTemp);
  Serial.print("C, ");
  Serial.print(dhtHumidity);
  Serial.println("%RH");
  Serial.print("Light: ");
  Serial.print(lightVal);
  Serial.println("%");
  Serial.print("DS18B20: ");
  Serial.print(dsTemp);
  Serial.println("C");
}

void debugGpsPrint() {
  Serial.print("GPS: ");
  Serial.print(gpsDate);
  Serial.print(",");
  Serial.print(gpsTime);
  Serial.print(" ");
  Serial.print(gpsLat, 6);
  Serial.print(",");
  Serial.print(gpsLng, 6);
  Serial.print(" ");
  Serial.print("Sats=");
  Serial.print(gpsSats);
  Serial.print(" ");
  Serial.print("HDOP=");
  Serial.print(gpsHdop);
  Serial.print(" ");
  Serial.print(gpsAlt);
  Serial.print("m ");
  Serial.print(gpsSpeed);
  Serial.print("m/s ");
  Serial.print(gpsCourse);
  Serial.println("deg");
  Serial.print(gpsDistAway);
  Serial.print("m ");
  Serial.print(gpsCourseTo);
  Serial.println("deg");
  //Serial.print(" (");
  //Serial.print(gpsCourseToText);
  //Serial.println(")");
}

void smsHandler(char smsMessageRaw[], bool execCommand, bool smsStartup) {
  //Serial.print("smsMessageRaw: ");
  //Serial.println(smsMessageRaw);

  char smsRecNumber[16];
  char smsMessage[16];

  int smsMessageRawSize = sizeof(smsMessageRaw);
  int numIndex, smsIndex;
  int x;
  for (x = 0   ; ; x++) {
    char c = smsMessageRaw[x];
    if (c == '"') {
      numIndex = x + 3;
      //Serial.println(numIndex);
      break;
    }
  }
  x++;
  //for (int x = smsMessageRawSize; ; x--) {
  for (x; ; x++) {
    char c = smsMessageRaw[x];
    if (c == '"') {
      smsIndex = x + 29;
      //Serial.println(smsIndex);
      break;
    }
  }

  x = 0;
  for (numIndex; ; numIndex++) {
    char c = smsMessageRaw[numIndex];
    if (c == '"') {
      smsRecNumber[x] = '\0';
      break;
    }
    smsRecNumber[x] = c;
    x++;
  }
  smsRecNumber[x] = '\0';
  //Serial.println(smsRecNumber);

  x = 0;
  for (smsIndex; ; smsIndex++) {
    char c = smsMessageRaw[smsIndex];
    if (c == '\n' || c == '\r') {
      smsMessage[x] = '\0';
      break;
    }
    smsMessage[x] = c;
    x++;
  }
  //Serial.println(smsMessage);

  if (!debugMode) {
    char debugString[64];
    char debugPrefix[] = "Received SMS message: ";
    sprintf(debugString, debugPrefix);
    strcat(debugString, smsMessage);
    char leftParenChar[] = " (";
    char rightParenChar[] = ")";
    strcat(debugString, leftParenChar);
    strcat(debugString, smsRecNumber);
    strcat(debugString, rightParenChar);
    logDebug(debugString);
  }

  if (smsStartup) {
    //if (smsMessage == "Ready") smsReadyReceived = true;
    char readyMessage[] = "Ready";
    for (x = 0; x < 5; x++) {
      if (smsMessage[x] == readyMessage[x]) smsReadyReceived = true;
      else {
        smsReadyReceived = false;
        break;
      }
    }
    if (!smsReadyReceived) startupFailure();
  }

  else if (execCommand) {
    int smsCommand = 0;

    //if (smsMessage.length() == 1) smsCommand = smsMessage.toInt();
    /*int messageLength = 0;
      for (x = 0; x < sizeof(smsMessage); x++) {
      if (isAlphaNumeric(smsMessage[x])) messageLength++;
      }
      if (messageLength == 1) smsCommand = int(smsMessage[0]);
      else; // Send SMS stating invalid command received (to incoming number)*/

    smsCommand = int(smsMessage[0]);
    //if (debugMode) {
    //Serial.print("smsCommand: ");
    //Serial.println(smsCommand);
    //}

    switch (smsCommand) {
      // LED
      case 49:  // ASCII character '1' = 49
        if (debugMode) Serial.print("SMS command #1 issued...");

        for (int x = 0; x < 5; x++) {
          digitalWrite(programStartLED, LOW);
          delay(100);
          digitalWrite(programStartLED, HIGH);
          delay(100);
        }
        break;

      // Location (Google Maps link sent via SMS)
      case 50:  // ASCII character '2' = 50
        if (debugMode) Serial.print("SMS command #2 issued...");

        Serial1.print("AT+CMGS=\"");
        Serial1.print(smsTargetNum);
        Serial1.println("\"");
        delay(100);
        Serial1.print("http://maps.google.com/maps?q=HAB@");
        Serial1.print(gpsLat, 6);
        Serial1.print(",");
        Serial1.print(gpsLng, 6);
        Serial1.println("&t=h&z=19&output=html");
        delay(100);
        Serial1.println((char)26);
        delay(100);
        smsFlush();
        smsMarkFlush = true;
        break;

      // Buzzer
      case 51:  // ASCII character '3' = 51
        if (debugMode) Serial.print("SMS command #3 issued...");
        digitalWrite(buzzerRelay, HIGH);
        buzzerStart = millis();
        break;
      default:
        if (debugMode) {
          Serial.print("INVALID COMMAND ISSUED: ");
          Serial.print(smsMessage);
        }
        break;
    }

    if (smsCommand == 49) sprintf(smsCommandText, "LED");
    else if (smsCommand == 51) sprintf(smsCommandText, "Buzzer");

    if (!debugMode) {
      char debugChar[64];
      char debugPrefix[] = "SMS command issued: ";
      char smsCommandChar[6];
      sprintf(debugChar, debugPrefix);
      sprintf(smsCommandChar, "%i", smsCommand);
      strcat(debugChar, smsCommandChar);
      logDebug(debugChar);
    }
  }
  else {
    if (debugMode) {
      Serial.print("smsMessage: ");
      Serial.println(smsMessage);
    }
  }
}

void smsMenu() {
  Serial1.print("AT + CMGS = \"");
  Serial1.print(smsTargetNum);
  Serial1.println("\"");
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
  Serial1.print("AT + CMGS = \"");
  Serial1.print(smsTargetNum);
  Serial1.println("\"");
  delay(100);
  Serial1.print(smsCommandText);
  Serial1.println(" activated.");
  delay(100);
  Serial1.println((char)26);
  delay(100);
  smsFlush();
  for (int x = 0; x < sizeof(smsCommandText); x++) {
    smsCommandText[x] = '\0';
  }
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

/* ----
   RTTY
   ---- */

void rttyProcessTx() {
  char rttyTxString[128];
  char gpsLatChar[10];
  char gpsLngChar[10];
  char gpsAltChar[10];
  char gpsSpeedChar[10];
  char gpsCourseChar[10];
  char dofAltChar[10];
  char dofTempChar[10];
  char dhtTempChar[10];
  char dhtHumidityChar[10];
  char commaChar[] = ",";

  sprintf(rttyTxString, callsignHeader);
  strcat(rttyTxString, commaChar);
  strcat(rttyTxString, auxLoopCountChar);
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
  strcat(rttyTxString, commaChar);
  dtostrf(dofTemp, 2, 2, dofTempChar);
  strcat(rttyTxString, dofTempChar);
  strcat(rttyTxString, commaChar);
  dtostrf(dhtTemp, 2, 2, dhtTempChar);
  strcat(rttyTxString, dhtTempChar);
  strcat(rttyTxString, commaChar);
  dtostrf(dhtHumidity, 2, 2, dhtHumidityChar);
  strcat(rttyTxString, dhtHumidityChar);
  strcat(rttyTxString, commaChar);

  unsigned int CHECKSUM = rttyCRC16Checksum(rttyTxString);
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(rttyTxString, checksum_str);

  rttyTxData(rttyTxString);
}

void rttyTxData (char *string) {
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
  if (bit) digitalWrite(rttyTxPin, HIGH);
  else digitalWrite(rttyTxPin, LOW);

  delayMicroseconds(3370); // 300 baud
  //delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
  //delayMicroseconds(10150); // 20150 doesn't work -- 16383 is the largest value Arduino can handle.
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

/* ---------
   HEARTBEAT
   ---------  */

void heartbeat() {
  digitalWrite(heartbeatOutputPin, HIGH);
  delay(10);
  digitalWrite(heartbeatOutputPin, LOW);
}

/* -----
   DEBUG
   -----  */

void debugBlink() {
  for (int x = 0; x < 3; x++) {
    delay(250);
    digitalWrite(debugLED, HIGH);
    delay(250);
    digitalWrite(debugLED, LOW);
  }
}

void startupFailure() {
  while (true) {
    digitalWrite(gpsReadyLED, LOW);
    digitalWrite(programStartLED, HIGH);
    delay(100);
    digitalWrite(gpsReadyLED, HIGH);
    digitalWrite(programStartLED, LOW);
    delay(100);
  }
}
