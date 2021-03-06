#!/usr/bin/env python

# TO DO
# - ????
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
import serial
import subprocess
from time import sleep
from timeit import default_timer as timer

habSerial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# NEED TO FLUSH ALL INCOMING SERIAL DATA HERE!!!!!!!!!!!!!!!!

captureInterval = 10
takeoffBreakTime = 600
#takeoffBreakTime = 180	# DEBUG VALUE
peakBreakTime = 900
#peakBreakTime = 240	# DEBUG VALUE
landingBreakTime = 7200
#landingBreakTime = 240	# DEBUG VALUE

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


def takeoff_capture():
    global videoStart

    # CONTINUE CAPTURING DOWN-FACING WEBCAM VIDEO UNTIL THRESHOLD ALTITUDE ACHEIVED
    startTime = timer()
    startTimeStatic = startTime
    continueCapture = True

    while continueCapture is True:
        videoStart = timer()
        capture_video(camDown, 120)
        sleep(1)
        
        while (timer() - startTime) <= 120:
            if habSerial.inWaiting() > 0:
                habOutput = habSerial.readline()[:-2]
                if habOutput:
                    if habOutput[0] == '$':
                        print 'Command received (T)'
                        print habOutput
                        #habCommand = serial_receive(habOutput)
                        if habOutput[0] == '0':
                            print 'RPi: Phase termination signal received.'
                            continueCapture = False
                        else:
                            print 'INVALID COMMAND RECEIVED.'
                    else:
                        print habOutput
                            
            capture_photo(camPi)
            #sleep(1)
            #capture_photo(camUp)
            sleep(10)

        print 'Down-facing video capture finished.'
        startTime = timer()
        if (startTime - startTimeStatic) > takeoffBreakTime:
            continueCapture = False


def peak_capture():
    # CONTINUE CAPTURING UP-FACING WEBCAM VIDEO UNTIL DESCENT DETECTED (?+10sec?)
    startTime = timer()
    startTimeStatic = startTime
    continueCapture = True

    while continueCapture is True:
        capture_video(camUp, 120)
        sleep(1)

        while (timer() - startTime) <= 120:
            if habSerial.inWaiting() > 0:
                habOutput = habSerial.readline()[:-2]
                if habOutput:
                    if habOutput[0] == '$':
                        print 'Command received (P)'
                        print habOutput
                        #habCommand = serial_receive(habOutput)
                        if habOutput[1] == '0':
                            print 'RPi: Phase termination signal received.'
                            continueCapture = False
                        else:
                            print 'INVALID COMMAND RECEIVED.'
                        #elif habCommand == '-1':
                            #print 'INVALID COMMAND RECEIVED.'
                    else:
                        print habOutput
                        
            capture_photo(camPi)
            #sleep(1)
            #capture_photo(camDown)
            sleep(10)

        print 'Up-facing video capture finished.'
        startTime = timer()
        if (startTime - startTimeStatic) > takeoffBreakTime:
            continueCapture = False


def landing_capture():
    # BEGIN CAPTURE OF DOWN-FACING WEBCAM VIDEO WHEN CLOSE TO GROUND AND CONTINUE UNTIL STATIONARY
    startTime = timer()
    startTimeStatic = startTime
    continueCapture = True

    while continueCapture is True:
        capture_video(camUp, 10)
        sleep(11)
        print 'Up-facing video capture finished.'
        capture_video(camDown, 120)
        sleep(1)

        while (timer() - startTime) <= 120:
            if habSerial.inWaiting() > 0:
                habOutput = habSerial.readline()[:-2]
                if habOutput:
                    if habOutput[0] == '$':
                        print 'Command received (L)'
                        print habOutput
                        #habCommand = serial_receive(habOutput)
                        if habOutput[1] == '0':
                            print 'RPi: Phase termination signal received.'
                            continueCapture = False
                        else:
                            print 'INVALID COMMAND RECEIVED.'
                        #elif habCommand == '-1':
                            #print 'INVALID COMMAND RECEIVED.'
                    else:
                        print habOutput
                    
            capture_photo(camPi)
            #sleep(1)
            #capture_photo(camUp)
            sleep(10)

        print 'Down-facing video capture finished.'
        startTime = timer()
        if (startTime - startTimeStatic) > takeoffBreakTime:
            continueCapture = False


programStart = False
while programStart is False:
    if habSerial.inWaiting() > 0:
        habOutput = habSerial.readline()[:-2]
        if habOutput:
            if habOutput[0] == '$':
                print 'Command received (P)'
                print habOutput
                #habCommand = serial_receive(habOutput)
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


while True:
    if (timer() - videoStart) > 120:
        # Will prevent read of incoming serial data
        print '--> MAIN PHOTO CAPTURE <--'
        capture_photo(camPi)
        sleep(1)
        capture_photo(camDown)
        sleep(1)
        capture_photo(camUp)
        sleep(1)
    else:
        print 'Waiting for video capture to complete.'
    
    startTime = timer()
    while (timer() - startTime) < captureInterval:
        if (timer() - videoStart) > 120:
            if habSerial.inWaiting() > 0:
                habOutput = habSerial.readline()[:-2]
                if habOutput:
                    if habOutput[0] == '$':
                        print 'Command received (M)'
                        print habOutput
                        #habCommand = serial_receive(habOutput)
                        if habOutput[1] == '1':
                            print '---> ENTERING TAKEOFF CAPTURE <--'
                            takeoff_capture()
                            print '---> EXITING TAKEOFF CAPTURE <--'
                        elif habOutput[1] == '2':
                            print '---> ENTERING PEAK CAPTURE <--'
                            peak_capture()
                            print '---> EXITING PEAK CAPTURE <--'
                        elif habOutput[1] == '3':
                            print '---> ENTERING LANDING CAPTURE <--'
                            landing_capture()
                            print '---> EXITING LANDING CAPTURE <--'
                    else:
                        print habOutput
