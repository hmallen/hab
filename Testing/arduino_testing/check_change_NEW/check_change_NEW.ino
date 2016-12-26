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
    if (!launchCapture) {
      // START LAUNCH CAP --> resetHandler = true ["2"]
      Serial.println("$1");
      digitalWrite(gpsReadyLED, HIGH);
      launchCapture = true;
      resetHandler = true;
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
          photoDeployStart = millis();
          peakCapture = true;
          resetHandler = true;
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
          Serial.println("$0");
          digitalWrite(programStartLED, LOW);
          EEPROM.write(1, 1);
          if (debugMode) {
            Serial.println();
            Serial.println("Descent phase triggered.");
            Serial.println();
          }
          resetHandler = false;
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
    if (!selfieRetract) {
      digitalWrite(photoDeployPin, HIGH);
      selfieRetract = true;
    }

    if (gpsAltChange > GPSCHANGETHRESHOLD) gpsChanges = 0;
    else if (gpsAltChange <= GPSCHANGETHRESHOLD) gpsChanges++;
    if (dofAltChange > DOFALTCHANGETHRESHOLD) dofChanges = 0;
    else if (dofAltChange <= DOFALTCHANGETHRESHOLD) dofChanges++;
    if (ms5607PressChange > BAROPRESSCHANGETHRESHOLD) ms5607Changes = 0;
    else if (ms5607PressChange <= BAROPRESSCHANGETHRESHOLD) ms5607Changes++;

    // LANDING CAPTURE BEGINS WHEN BELOW THRESHOLD ALTITUDE
    if (!resetHandler && !landingCapture) {
      // WAIT FOR ALTITUDE BELOW THRESHOLD
      //// BEGIN LANDING CAP --> landingCapture = true --> resetHandler = true
      if ((dofAlt - dofAltOffset) < LANDINGCAPTURETHRESHOLD || !debugState) {
        if (!debugState) debugBlink();
        Serial.println("$3");
        digitalWrite(gpsReadyLED, HIGH);
        landingCapture = true;
        resetHandler = true;
      }
    }

    // LANDING PHASE BEGINS WHEN ALTITUDE STOPS CHANGING
    else if (resetHandler && landingCapture) {
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
    Serial.println("$0");
    digitalWrite(gpsReadyLED, LOW);
    resetHandler = false;

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
    Serial1.print(" from launch site.");
    Serial1.print(": http://maps.google.com/maps?q=HAB@");
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
