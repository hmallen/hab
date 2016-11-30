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
import RPi.GPIO
import subprocess
import sys
from time import sleep

camPi = 'rpi'
camDown = 'down'
camUp = 'up'
camera = picamera.PiCamera()

gpio = RPi.GPIO()

inputStart = 17
inputPeak = 18
inputLanding = 19
gpioInputs = [inputStart, inputPeak, inputLanding]

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)
gpio.setup(gpioInputs, gpio.IN, pull_up_down=gpio.PUD_DOWN)


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


while not gpio.input(inputStart):
    sleep(1)

capture_photo(0)
sleep(30)
# DO STUFF & THINGS

while True:
    print 'TEST'  # MAIN LOOP POST-TAKEOFF VIDEO RECORDING
