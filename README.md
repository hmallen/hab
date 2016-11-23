# High-Altitude Balloon

TO DO:
- Mount RTTY board
- Wire power: 1)RTTY 2)Buzzer 3)Relays
- Add webcam to lid (port usb through to RPi)
- Create new RPi script for photos/videos only (RTTY TX moved to Arduino)
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
