#!/usr/env/python

# TO DO
#
# Add outputs to Arduino sketch
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

from picamera import PiCamera
from time import sleep
import sys
import subprocess
import RPi.GPIO as gpio

inputStart = 17
inputPeak = 18
inputLanding = 19
gpioInputs = [inputStart, inputPeak, inputLanding]

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)
gpio.setup(gpioInputs, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

camera = PiCamera()

def capture_photo():
    camera.start_preview(2)
    camera.capture('test.jpg')

while (!gpio.input(inputStart)):
    sleep(1)




capture_photo()
sleep(30)
# DO STUFF & THINGS