#!/usr/bin/env python

import datetime
import picamera
import serial
import subprocess
from time import sleep
from timeit import default_timer as timer
import sys

habSerial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

camPi = 'rpi'
camDown = 'down'
camUp = 'up'

camera = picamera.PiCamera()
camera.resolution = (2592, 1944)
camera.framerate = 15

global videoStartDown, videoStartUp
videoStartDown = -120
videoStartUp = -120
camDownActive = False
camUpActive = False

captureInterval = 30
takeoffMaxDuration = 600
peakMaxDuration = 900
landingMaxDuration = 7200

takeoffStart = 0
peakStart = 0
landingStart = 0

######## MODE #########
# 0 = Regular capture #
# 1 = Takeoff         #
# 2 = Peak            #
# 3 = Landing         #
#######################
programMode = 0


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
    global videoStartDown, videoStartUp

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

        videoStartUp = timer()

    elif camType == 'down':
        print 'Down-facing video capture started.'
        popenString = './webcam_video.sh 1 ' + str(vidLength)
        popenCommand = subprocess.Popen([popenString], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        std_out, std_err = popenCommand.communicate()
        status = std_out.strip('\n')
        error = std_err.strip('\n')

        videoStartDown = timer()

try:
    while habSerial.inWaiting() > 0:
        habSerial.readline()
        sleep(0.005)

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
                else:
                    print habOutput

    while True:
        loopStart = timer()

        while (timer() - loopStart) < captureInterval:
            # Regular serial reads to read incoming commands
            if habSerial.inWaiting() > 0:
                habOutput = habSerial.readline()[:-2]
                if habOutput:
                    if habOutput[0] == '$':
                        print 'Command received (Main): ' + habOutput
                        if habOutput[1] == '0':
                            programMode = 0
                            print 'Entering main phase.'
                        elif habOutput[1] == '1':
                            programMode = 1
                            takeoffStart = timer()
                            print 'Entering takeoff capture.'
                        elif habOutput[1] == '2':
                            programMode = 2
                            peakStart = timer()
                            print 'Entering peak capture.'
                        elif habOutput[1] == '3':
                            programMode = 3
                            landingStart = timer()
                            print 'Entering landing capture.'
                        elif habOutput[1] == '4':
                            programMode = 4
                            captureInterval = 180
                            print 'Entering landing phase.'
                    else:
                        print habOutput

        # Mode-specific program functions
        if programMode == 0:    # Regular capture
            if (timer() - videoStartDown) > 120 and camDownActive == True:
                camDownActive = False
                print 'Down-facing video capture finished.'
            if (timer() - videoStartUp) > 120 and camUpActive == True:
                camUpActive = False
                print 'Up-facing video capture finished.'

            if camDownActive == False:
                capture_photo(camDown)
                sleep(1)
            if camUpActive == False:
                capture_photo(camUp)
                sleep(1)

        elif programMode == 1:  # Takeoff capture
            if (timer() - videoStartDown) > 120 and camDownActive == True:
                camDownActive = False
                print 'Down-facing video capture finished.'
            if (timer() - videoStartUp) > 120 and camUpActive == True:
                camUpActive = False
                print 'Up-facing video capture finished.'

            if camDownActive == False and camUpActive == False:
                capture_video(camDown, 120)
                sleep(1)
                camDownActive = True
            if camUpActive == False:
                capture_photo(camUp)
                sleep(1)

            if (timer() - takeoffStart) > takeoffMaxDuration:
                programMode = 0
                print 'Takeoff timeout. Entering main phase.'

        elif programMode == 2:  # Peak capture
            if (timer() - videoStartDown) > 120 and camDownActive == True:
                camDownActive = False
                print 'Down-facing video capture finished.'
            if (timer() - videoStartUp) > 120 and camUpActive == True:
                camUpActive = False
                print 'Up-facing video capture finished.'

            if camUpActive == False and camDownActive == False:
                capture_video(camUp, 120)
                sleep(1)
                camUpActive = True
            if camDownActive == False:
                capture_photo(camDown)
                sleep(1)

            if (timer() - peakStart) > peakMaxDuration:
                programMode = 0
                print 'Peak timeout. Entering main phase.'

        elif programMode == 3:  # Landing capture
            if (timer() - videoStartUp) > 10 and camUpActive == False and camDownActive == False:
                capture_video(camUp, 10)
                sleep(1)
                camUpActive = True
            elif (timer() - videoStartUp) > 10 and camUpActive == True:
                camUpActive = False
                print 'Up-facing video capture finished.'

            if (timer() - videoStartDown) > 120 and camDownActive == False and camUpActive == False:
                capture_video(camDown, 120)
                sleep(1)
                camDownActive = True
            elif (timer() - videoStartDown) > 120 and camDownActive == True:
                camDownActive = False
                print 'Down-facing video capture finished.'

            if camDownActive == False:
                capture_photo(camDown)
                sleep(1)
            elif camUpActive == False:
                capture_photo(camUp)
                sleep(1)

            if (timer() - landingStart) > landingMaxDuration:
                programMode = 4
                captureInterval = 180
                print 'Landing timeout. Entering main phase.'

        elif programMode == 4:  # Landing phase
            if (timer() - videoStartDown) > 120 and camDownActive == True:
                camDownActive = False
                print 'Down-facing video capture finished.'
            if (timer() - videoStartUp) > 120 and camUpActive == True:
                camUpActive = False
                print 'Up-facing video capture finished.'

            if camDownActive == False:
                capture_photo(camDown)
                sleep(1)
            if camUpActive == False:
                capture_photo(camUp)
                sleep(1)

        capture_photo(camPi)
        sleep(1)

except (KeyboardInterrupt, SystemExit):
    while habSerial.inWaiting() > 0:
        habSerial.readline()
    habSerial.close()
    sys.exit()
