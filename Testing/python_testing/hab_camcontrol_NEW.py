#!/usr/bin/env python

import datetime
import picamera
import serial
import subprocess
from time import sleep
from timeit import default_timer as timer

habSerial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

camPi = 'rpi'
camDown = 'down'
camUp = 'up'
camera = picamera.PiCamera()
camera.resolution = (2592, 1944)
camera.framerate = 15

global videoStart

videoStart = -120


def capture_photo(camType):
    if camType == 'rpi':
        print 'PiCam photo capture started.'

        timestamp = datetime.datetime.now().strftime("%m%d%Y-%H%M%S")
        filename = 'media/photos/RPI-' + timestamp + '.jpg'
        camera.start_preview()
        sleep(2)
        camera.capture(filename)
        camera.stop_preview()

        #popenString = './rpi_photo.sh'
        #popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        #std_out, std_err = popenCommand.communicate()
        #status = std_out.strip('\n')
        #error = std_err.strip('\n')

        print 'PiCam photo capture finished.'
    elif camType == 'up':
        print 'Up-facing photo capture started.'
        popenString = './webcam_photo.sh 0'
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')
        print 'Up-facing photo capture finished.'
    elif camType == 'down':
        print 'Down-facing photo capture started.'
        popenString = './webcam_photo.sh 1'
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')
        print 'Down-facing photo capture finished.'


def capture_video(camType, vidLength):
    if camType == 'rpi':
        print 'PiCam video capture started.'

        timestamp = datetime.datetime.now().strftime("%m%d%Y-%H%M%S")
        filename = 'media/videos/RPI-' + timestamp + '.h264'
        camera.start_preview()
        sleep(2)
        camera.start_recording(filename)
        camera.wait_recording(vidLength)
        camera.stop_preview()
        camera.stop_recording()

        #popenString = './rpi_video.sh ' + str(vidLength)
        #popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        #std_out, std_err = popenCommand.communicate()
        #status = std_out.strip('\n')
        #error = std_err.strip('\n')

        print 'PiCam video capture finished.'
    elif camType == 'up':
        print 'Up-facing video capture started.'
        popenString = './webcam_video.sh 0 ' + str(vidLength)
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')
    elif camType == 'down':
        print 'Down-facing video capture started.'
        popenString = './webcam_video.sh 1 ' + str(vidLength)
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')


while habSerial.inWaiting() > 0:
    habSerial.readline()
    

programStart = False
while programStart is False:
    if habSerial.inWaiting() > 0:
        habOutput = habSerial.readline()[:-2]
        if habOutput:
            if habOutput[0] == '$':
                print 'Command received (Start): ' + habOutput
                if habOutput[1] == '0':
                    habSerial.write('$0')
                    programStart = True
                    break
                else:
                    print 'INVALID COMMAND RECEIVED.'
                #elif habCommand == '-1':
                    #print 'INVALID COMMAND RECEIVED.'
            else:
                print habOutput
