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
import picamera
#import RPi.GPIO
import serial
import subprocess
from time import sleep
from timeit import default_timer as timer

habSerial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

captureInterval = 30
takeoffBreakTime = 600

camPi = 'rpi'
camDown = 'down'
camUp = 'up'
camera = picamera.PiCamera()

#gpio = RPi.GPIO()

#gpioInput = 17

#gpio.setwarnings(False)
#gpio.setmode(gpio.BOARD)
#gpio.setup(gpioInput, gpio.IN, pull_up_down=gpio.PUD_DOWN)


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
    print 'DO STUFF AND THINGS.'


def capture_photo(camType):
    if camType == 'rpi':
        timestamp = datetime.datetime.now().strftime("%m%d%Y-%H%M%S")
        filename = 'media/photos/RPI-' + timestamp + '.jpg'
        camera.start_preview()
        sleep(2)
        camera.capture(filename)
    elif camType == 'up':
        popenString = './webcam_photo.sh 0'
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')
    elif camType == 'down':
        popenString = './webcam_photo.sh 1'
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')


def capture_video(camType, vidLength):
    if camType == 'rpi':
        timestamp = datetime.datetime.now().strftime("%m%d%Y-%H%M%S")
        filename = 'media/videos/RPI-' + timestamp + '.h264'
        camera.start_recording(filename)
        camera.wait_recording(vidLength)
        camera.stop_recording()
    elif camType == 'up':
        popenString = './webcam_video.sh 0' + ' ' + str(vidLength)
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')
    elif camType == 'down':
        popenString = './webcam_video.sh 1' + ' ' + str(vidLength)
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')


def takeoff_capture():
    startTime = timer()
    startTimeStatic = startTime
    while True:
        habOutput = habSerial.readline()[:-2]
        if habOutput:
            habCommand = parse_output(habOutput)
            print habCommand
            if habCommand == '0':
                break
        capture_video(camDown, 60)
        while (timer() - startTime) <= 60:
            sleep(1)
            capture_photo(camPi)
            sleep(1)
            capture_photo(camUp)
            sleep (10)
        startTime = timer()
        if (startTime - startTimeStatic) > takeoffBreakTime:
            break


def peak_capture():
    print 'Peak capture.'


def landing_capture():
    print 'Landing capture.'


while True:
    habOutput = habSerial.readline()[:-2]
    if habOutput:
        habCommand = parse_output(habOutput)
        print habCommand
        if habCommand == '0':
            habSerial.write('$0')
            break

while True:
    capture_photo(camPi)
    sleep(1)
    capture_photo(camDown)
    sleep(1)
    capture_photo(camUp)
    sleep(1)
    
    startTime = timer()
    while (timer() - startTime) < captureInterval:
        habOutput = habSerial.readline()[:-2]
        if habOutput:
            habCommand = parse_output(habOutput)
            print habCommand
            if habCommand == '1':
                takeoff_capture()
            elif habCommand == '2':
                peak_capture()
            elif habCommand == '3':
                landing_capture()
