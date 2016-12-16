#!/usr/bin/env python

with open('mode_file.txt', 'w') as mode_file:
    mode_file.write('3')

state = mode_file.closed
print state
