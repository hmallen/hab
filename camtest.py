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
import subprocess
from time import sleep

camDown = '/dev/video0'  # CHECK THAT THIS IS CORRECT
camUp = '/dev/video1'    # CHECK THAT THIS IS CORRECT
camera = picamera.PiCamera()


def capture_photo(camType):
    if camType == 'rpi':
        timestamp = datetime.datetime.now().strftime("%m%d%Y-%H%M%S")
        filename = '~/icarus_one/media/photos/RPI-' + timestamp + '.jpg'
        camera.start_preview(2)
        camera.capture(filename)
    elif camType == 'up':
        popenCommand = subprocess.Popen('./up_photo.sh', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')
    elif camType == 'down':
        popenCommand = subprocess.Popen('./down_photo.sh', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')


def capture_video(camType):
    if camType == 'rpi':
        timestamp = datetime.datetime.now().strftime("%m%d%Y-%H%M%S")
        filename = '~/icarus_one/media/photos/RPI-' + timestamp + '.mp4'
        camera.start_recording(filename)
        camera.wait_recording(5)
        camera.stop_recording()
    elif camType == 'up':
        popenCommand = subprocess.Popen('./up_video.sh', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')
    elif camType == 'down':
        popenCommand = subprocess.Popen('./down_video.sh', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')


capture_photo('rpi')
capture_photo('up')
capture_photo('down')
capture_video('rpi')
capture_video('up')
capture_video('down')

while True:
    sleep(60)
