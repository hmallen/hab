#!/usr/bin/env bash

FILEINPUT=$1
FILEOUTPUT=$2

sudo avconv -i $FILEINPUT -preset ultrafast -crf 27 $FILEOUTPUT
