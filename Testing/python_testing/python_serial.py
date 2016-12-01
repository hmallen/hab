#!/usr/bin/env python

import serial
from time import sleep

habSerial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def parse_output(serialData):
    if serialData[0] == '$':
        if serialData[1] == '0':
            print '0'
            return '0'
        elif serialData[1] == '1':
            print '1'
            return '1'
        elif serialData[1] == '2':
            print '2'
            return '2'
        elif serialData[1] == '3':
            print '3'
            return '3'
        else:
            print serialData
            return 'X'
    else:
        print 'Invalid command.'
        return 'Y'

while True:
    #try:
    #    habOutput = habSerial.readline()[:-2]
    #    if habOutput:
    #        habCommand = parse_output(habOutput[1:])
    #        print habCommand
    #    else:
    #        print 'Not a data string!'
    #except:
    #    continue  # Can place regular functions here

    habOutput = habSerial.readline()[:-2]
    if habOutput:
        habCommand = parse_output(habOutput)
        print habCommand
        if habCommand == '0':
            sleep(5)
            habSerial.write('Hello!')
    
    sleep(1)
