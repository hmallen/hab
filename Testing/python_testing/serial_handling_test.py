#!/usr/bin/env python

# TO DO
#
# Add outputs to Arduino sketch
# Test if PiCamera capture possible while recording video
# Test if 2 videos can be recorded simultaneously
#
# Photo:
# - Regular capture from PiCam
# - Regular capture from down-facing webcam
# - Regular capture from up-facing webcam
#
# Video:
# - Capture from down-facing webcam during takeoff phase
# - Capture from up-facing webcam when approaching peak through ~+0:60 seconds
#

import datetime
import serial
from time import sleep
from timeit import default_timer as timer

habSerial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

captureInterval = 30
takeoffBreakTime = 600
peakBreakTime = 900
landingBreakTime = 7200


def serial_receive(serialData):
    if serialData[0] == '$':
        if serialData[1] == '0':
            return '0'
        elif serialData[1] == '1':
            return '1'
        elif serialData[1] == '2':
            return '2'
        elif serialData[1] == '3':
            return '3'
        else:
            print 'Invalid serial command received.'
            return '-1'


def serial_send(serialData):
    habSerial.write(serialData)


def takeoff_capture():
    # CONTINUE CAPTURING DOWN-FACING WEBCAM VIDEO UNTIL THRESHOLD ALTITUDE ACHEIVED
    startTime = timer()
    startTimeStatic = startTime
    while True:
        habOutput = habSerial.readline()[:-2]
        if habOutput:
            habCommand = serial_receive(habOutput)
            print habCommand
            if habCommand == '0':
                break
        capture_video(camDown, 120)
        sleep(1)
        while (timer() - startTime) <= 120:
            sleep(1)
            capture_photo(camPi)
            sleep(1)
            capture_photo(camUp)
            sleep (10)
        startTime = timer()
        if (startTime - startTimeStatic) > takeoffBreakTime:
            break


while True:
    if habSerial.inWaiting() > 0:
        while habSerial.inWaiting() > 0:
            habOutput = habSerial.readline()[:-2]
            if habOutput[0] == '$':
                habCommand = serial_receive(habOutput)
                print habCommand
                if habCommand == '0':
                    #serial_send('$0')
                    break
            else:
                print habOutput

while True:
    startTime = timer()
    while (timer() - startTime) < captureInterval:
        habOutput = habSerial.readline()[:-2]
        if habOutput:
            habCommand = serial_receive(habOutput)
            print habCommand
            if habCommand == '1':
                print 'takeoff_capture()'
            elif habCommand == '2':
                print 'peak_capture()'
            elif habCommand == '3':
                print 'landing_capture()'
