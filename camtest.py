#!/usr/env/python

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

camDown = '/dev/video0'  # CHECK THAT THIS IS CORRECT
camUp = '/dev/video1'    # CHECK THAT THIS IS CORRECT
camera = PiCamera()


def capture_photo(cam):
    if cam == 0:
        camera.start_preview(2)
        camera.capture('test.jpg')
    elif cam == 1:
        print 'TEST'  # FSWEBCAM (camDown)
    elif cam == 2:
        print 'TEST'  # FSWEBCAM (camUp)


def capture_video(cam):
    if cam == 0:
        print 'TEST'  # RASPIVID
    elif cam == 1:
        print 'TEST'  # AVCONV (camDown)
    elif cam == 2:
        print 'TEST'  # AVCONV (camUp)


capture_photo(0)
sleep(30)
# DO STUFF & THINGS

popen_string = 'python exchange_lorenz_tradeexecution_v2.py -s buy'
order = subprocess.Popen([popen_string], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
std_out, std_err = order.communicate()
status = std_out.strip('\n')
error = std_err.strip('\n')
