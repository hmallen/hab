#!/usr/bin/env python

import RPi.GPIO as GPIO
from time import sleep

gpioInputs = [15, 16]

gpio.setwarnings(False)
gpio.setmode(GPIO.BOARD)
gpio.setup(gpioInputs, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while (True):
        fifteen = GPIO.input(15)
        sixteen = GPIO.input(16)
        pinString = str(seventeen) + ' / ' + str(eighteen)
        print pinString
        sleep(0.5)
except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
    pwm.stop() # stop PWM
    GPIO.cleanup() # cleanup all GPIO
