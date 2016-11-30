#!/usr/bin/env python

input RPi.GPIO
from time import sleep

gpio = RPi.GPIO()

gpioInputs = [17, 18]

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)
gpio.setup(gpioInputs, gpio.IN, pull_up_down=gpio.PUD_DOWN)

while (True):
    seventeen = gpio.input(17)
    eighteen = gpio.input(18)
    pinString = str(seventeen) + ' / ' + str(eighteen)
    print pinString
    sleep(0.5)
