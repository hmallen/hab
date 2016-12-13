# High-Altitude Balloon

TO DO:
- Wire power: 1)RTTY 2)Buzzer 3)Relays
- Ensure that gas sensors receiving correct (5V) voltage [2.5V was measured after first battery test]
- Weigh full payload package
-- Determine hydrogen vs. helium w/ 600g balloon
-- Determine parachute diameter
- Test payload heater and gas sensors with battery arrays
-- Time & Stress testing (payload heater @ sustained high current)

Pre-launch Checklist:
- Burn-in gas sensors for 24-48 hours
- Set: debugMode = false
- Set: debugSmsOff = false
- Set: GASSENSORWARMUP to original duration
- Turn on roaming for GPRS
- Clear SD card

Considerations:
- Balloon-Payload attachment rope shouldn't rotate freely about z-axis

------------------------------------------------------------------------------------------------------------------

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
1 --> Pin 7 (Gas sensors)
2 --> Pin 6 (Internal payload heater)
3 --> Pin 5
4 --> Pin 4

Gas Sensors:
- MQ-2 --> A7
- MQ-3 --> A8
- MQ-4 --> A9
- MQ-5 --> A10
- MQ-6 --> A11
- MQ-7 --> A12
- MQ-8 --> A13
- MQ-9 --> A14
- MQ135 --> A15

EEPROM Values:
0 --> Setup complete
1 --> Descent phase
2 --> Landing phase

TO DO:
- MUST FIND FUNCTION TO CONFIRM GPRS POWER TO RESTART IF NECESSARY!!!! ****
- CHECK RTTY BROADCAST ****
- CHECK BATTERY POWER FUNCTION ****
- INTEGRATE ARDUINO-->RPI COMMUNICATION ****
- Confirm that GPS coordinates are sent with highest precision (i.e. 6 floating point decimals)
- Add check and retry for MS5607 data validity (every so often a bad value appears)
- Add DS1820B data validity check to prevent accidental relay tripr
-- Also add startup check function (i.e. In initSensors())
- On SMS startup, input current SLP to provide altimeter offset????
- Handling of gas sensor logging after sensor shut-off
- Determine pressure value to trigger peak capture (Reference press vs. alt table & balloon data)
- Create test interrupt functions for pins 44-47 (Trigger fake alt/checkChange()/etc values)
-- TEST INPUT VALUES FOR PIN (i.e. INPUT_PULLUP STATE @ 1/0)
- TEST ARDUINO-->RPI SERIAL COMMUNICATION TRIGGERS
- TURN ON ROAMING BEFORE LIVE LAUNCH TO ENSURE PRESENCE OF GPRS NETWORK CONNECTION
- CHANGE GAS SENSOR WARMUP BACK TO NORMAL BEFORE LIVE LAUNCH
- Add RTTY test TX before SMS confirmation?
- Add MS5607 altitude????
- Stop reading gas sensors after descent?
- Make sure calls for buzzer use new variable ("buzzerRelay")
- Add exception for sensor initialization if setupComplete is true??
- Change "ada..." to "dof" for consistency
- Add servo/picture taking function
- Confirm that boolean argument in smsPower function actually necessary
- Test if SMS stores in buffer when no signal and sends when reconnected
- Add check to confirm GPRS is powered on (Need to find suitable function)
-- Also check if similar function to indicate network connectivity
- Gather sender number and log incoming/outgoing SMS messages to SD card
- Consider setting sampling rate based on theoretical ascent rate
- Cut-down
- Change global variables to functions returning pointer arrays

CONSIDERATIONS:
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
