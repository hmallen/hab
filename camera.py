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

from picamera import PiCamera
from time import sleep
import subprocess
import RPi.GPIO

camDown = '/dev/video0'  # CHECK THAT THIS IS CORRECT
camUp = '/dev/video1'    # CHECK THAT THIS IS CORRECT
camera = PiCamera()

gpio = RPi.GPIO()

inputStart = 17
inputPeak = 18
inputLanding = 19
gpioInputs = [inputStart, inputPeak, inputLanding]

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)
gpio.setup(gpioInputs, gpio.IN, pull_up_down=gpio.PUD_DOWN)


def capture_photo(camType):
    if camType == 0:
        camera.start_preview(2)
        camera.capture('test.jpg')
    elif camType == 1:
        print 'TEST'  # FSWEBCAM (camDown)
    elif camType == 2:
        print 'TEST'  # FSWEBCAM (camUp)


def capture_video(camType):
    if camType == 0:
        print 'TEST'  # RASPIVID
    elif camType == 1:
        print 'TEST'  # AVCONV (camDown)
    elif camType == 2:
        print 'TEST'  # AVCONV (camUp)


while not gpio.input(inputStart):
    sleep(1)

capture_photo(0)
sleep(30)
# DO STUFF & THINGS

while True:
    print 'TEST'  # MAIN LOOP POST-TAKEOFF VIDEO RECORDING

popen_string = 'python exchange_lorenz_tradeexecution_v2.py -s buy'
order = subprocess.Popen([popen_string], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
std_out, std_err = order.communicate()
status = std_out.strip('\n')
error = std_err.strip('\n')
