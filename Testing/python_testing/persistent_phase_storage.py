#!/usr/bin/env python

with open('mode_file.txt', 'w') as mode_file:
    mode_file.write('-1')

state = mode_file.closed
print state

with open('mode_file.txt', 'r') as mode_file:
    programState = mode_file.read()

print programState
