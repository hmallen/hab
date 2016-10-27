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
   - Test if SMS stores in buffer when no signal and sends when reconnected
   - Set time using new library at program start then add time stamp to each log file write
   - Add GPS & Aux "update" booleans to logging to indicate new data
   - Log DOF and other data to separate log files
   - Change #define variables to ALL CAPS
   - Add check to confirm GPRS is powered on
   - Add SMS send/receive confirmation on program start
   -- Sends test SMS
   -- Require response for program start
   -- Power off GPRS when correct response received
   - Determine SMS functions and create menu
   - Log incoming/outgoing SMS messages to SD card
   - Change digital pin order for uniformity
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
   - Utilize TinyGPS++ libraries to calculate distance and course from home
   - Power SMS during ms5607 initialization and send test SMS

   CONSIDERATIONS:
   - Create bools for each sensor set and disable reads if failing
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
#include <MS5xxx.h>
#include <SdFat.h>
#include <SHT1x.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>

// Definitions
#define gpsHdopThreshold 200  // Change to 150 for live conditions
#define dofDataInterval 500 // Update interval (ms) for Adafruit 1604 data
#define auxDataInterval 5000 // Update interval (ms) for data other than that from Adafruit 1604
#define gpsDataInterval 15000 // Update interval (ms) for GPS data updates
#define smsPowerDelay 5000  // Delay between power up and sending of commands to GPRS

const bool debugMode = true;
const bool debugSmsOff = true;

// Digital Pins
const int chipSelect = SS;
const int shtData = 2;
const int shtClock = 3;
const int relayPin1 = 7;
const int relayPin2 = 6;
const int relayPin3 = 5;
const int relayPin4 = 4;
const int smsPowerPin = 22;
const int gpsPpsPin = 26;
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
bool smsReadyReceived = false;
bool smsFirstFlush = false;
bool gpsFix = false;
float gpsLat, gpsLng, gpsAlt, gpsSpeed;
float gpsLatLast, gpsLngLast, gpsAltLast, gpsSpeedLast;
uint32_t gpsDate, gpsTime, gpsSats, gpsCourse;
int32_t gpsHdop;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;  // Can this be a const float??
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float dofRoll, dofPitch, dofHeading;
float dofPressure, dofTemp, dofAlt;
float ms5607Temp, ms5607Press;
int gasValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float shtTemp, shtHumidity;
int lightVal;

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
    Serial.println("No MS5607 detected...Check your wiring!");
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

  bool gpsPower = false;
  bool gpsPpsVal;
  int gpsPpsPulses;

  while (true) {
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
    Serial.println("GPS powered.");
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
    Serial.println("GPS fix attained.");
    break;
  }
  digitalWrite(gpsReadyLED, HIGH);
}

void smsPower(bool powerState) {
  // Power on GPRS and set proper modes for operation
  if (powerState == true) {
    digitalWrite(smsPowerPin, HIGH);
    delay(500);
    digitalWrite(smsPowerPin, LOW);

    delay(smsPowerDelay);

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
  pinMode(relayPin1, OUTPUT); digitalWrite(relayPin1, LOW);
  pinMode(relayPin2, OUTPUT); digitalWrite(relayPin2, LOW);
  pinMode(relayPin3, OUTPUT); digitalWrite(relayPin3, LOW);
  pinMode(relayPin4, OUTPUT); digitalWrite(relayPin4, LOW);
  pinMode(smsPowerPin, OUTPUT); digitalWrite(smsPowerPin, LOW);
  pinMode(gpsReadyLED, OUTPUT); digitalWrite(gpsReadyLED, LOW);
  pinMode(programStartLED, OUTPUT); digitalWrite(programStartLED, LOW);
  pinMode(gpsPpsPin, INPUT_PULLUP);
  pinMode(programStartPin, INPUT_PULLUP);

  Serial.begin(9600); // Debug output
  Serial1.begin(19200); // GPRS communication
  Serial2.begin(9600);  // GPS communication

  Serial.println();
  Serial.print("Initializing SD card...");
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }
  Serial.println("complete.");

  Serial.println("Initializing sensors:");
  initSensors();
  Serial.println("Sensor initialization complete.");

  smsPower(true);
  delay(smsPowerDelay);

  for (int x = 0; x < 2; x++) {
    digitalWrite(programStartLED, HIGH);
    delay(250);
    digitalWrite(programStartLED, LOW);
    delay(250);
  }

  if (!debugSmsOff) {
    Serial1.print("AT+CMGS=\""); Serial1.print(smsTargetNum); Serial1.println("\"");
    delay(100);
    Serial1.println("Reply with \"Ready\" to continue.");
    delay(100);
    Serial1.println((char)26);
    delay(100);
    smsFlush();

    Serial.print("Waiting for SMS ready message...");
    smsConfirmReady();
    Serial.println("received.");
  }

  smsPower(false);

  for (int x = 0; x < 2; x++) {
    digitalWrite(programStartLED, HIGH);
    delay(250);
    digitalWrite(programStartLED, LOW);
    delay(250);
  }

  Serial.print("Flip switch to continue...");
  while (true) {
    if (digitalRead(programStartPin) == 1) {
      digitalWrite(programStartLED, HIGH);
      break;
    }
    delay(100);
  }
  Serial.println("starting program.");
  Serial.println();
}

void loop() {
  for (unsigned long startTimeGPS = millis(); (millis() - startTimeGPS) < gpsDataInterval; ) {
    for (unsigned long startTimeAux = millis(); (millis() - startTimeAux) < auxDataInterval; ) {
      readAda1604();

      Serial.print("DOF Loop #: "); Serial.println(dofLoopCount);

      if (debugMode) debugDofPrint();
      delay(dofDataInterval);

      dofLoopCount++;
    }
    delay(100);

    if (!readMS5607()) {
      Serial.println("Failed to read MS5607.");  // WRITE TO LOG FILE INSTEAD
      ms5607Temp = 0.0;
      ms5607Press = 0.0;
      break;
    }
    else break;

    readGas();
    readSht();
    readLight();

    Serial.println();
    Serial.print("Aux Loop #: "); Serial.println(auxLoopCount);
    auxLoopCount++;

    if (debugMode) debugAuxPrint();
    Serial.println();
  }

  readGps();

  Serial.print("GPS Loop #: "); Serial.println(gpsLoopCount);
  gpsLoopCount++;

  if (debugMode) debugGpsPrint();
  else logData();
  Serial.println();

  if (Serial1.available()) {
    Serial.println("Incoming GPRS serial data.");
    String smsMessageRaw = "";
    while (Serial1.available()) {
      char c = Serial1.read();
      Serial.write(c);
      smsMessageRaw += c;
      delay(10);
    }
    smsHandler(smsMessageRaw, false, false);
  }
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
    dofTemp = (float)temperature;
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

  if (gpsLat != gpsLatLast) gpsLatLast = gpsLat;
  if (gpsLng != gpsLngLast) gpsLatLast = gpsLat;
  if (gpsAlt != gpsAltLast) gpsLatLast = gpsLat;
  if (gpsSpeed != gpsSpeedLast) gpsLatLast = gpsLat;
}

bool readMS5607() {
  ms5607.ReadProm();
  ms5607.Readout();

  ms5607Temp = ms5607.GetTemp() / 100.0;
  ms5607Press = ms5607.GetPres() / 100.0;

  uint8_t crc4Calc = ms5607.Calc_CRC4();
  uint8_t crc4Read = ms5607.Read_CRC4();
  uint8_t crc4Code = ms5607.CRCcodeTest();
  uint8_t crc4Expected = 0xB;

  /*Serial.print(crc4Calc, HEX); Serial.print("/");
    Serial.print(crc4Read, HEX); Serial.print(" - ");
    Serial.print(crc4Code, HEX); Serial.print(" [");
    Serial.print(crc4Expected, HEX); Serial.println("]");*/

  if (crc4Read != crc4Calc || crc4Code != crc4Expected) {
    Serial.println("MS5607 CRC4 check failed.");
    Serial.print(crc4Calc, HEX); Serial.print("/");
    Serial.print(crc4Read, HEX); Serial.print(" - ");
    Serial.print(crc4Code, HEX); Serial.print(" [");
    Serial.print(crc4Expected, HEX); Serial.println("]");
    return false;
  }
  else return true;
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
  logFile.print(ms5607Press); logFile.print(",");
  logFile.print(ms5607Temp); logFile.print(",");
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
  logFile.flush();
  logFile.close();
}

void logOther(String dataString) {
  if (!logFile.open(debug_file, O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("Opening debug log for write failed.");
  }
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

  // LOG DATA HERE

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
      case 1:
        Serial.println("Command #1");
        break;
      case 2:
        // Activate buzzer
        Serial.println("Command #2");
        break;
      case 3:
        // Trigger smoke signal
        Serial.println("Command #3");
        break;
      default:
        Serial.print("INVALID COMMAND: ");
        Serial.println(smsMessage);
        break;
    }
    // LOG DATA HERE
  }
  else Serial.print("smsMessage: "); Serial.println(smsMessage);
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
