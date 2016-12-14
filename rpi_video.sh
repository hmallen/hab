#!/usr/bin/env bash

LENGTHSEC=$1
((LENGTH=LENGTHSEC*1000))

CAMTYPE="RPI-"

PREFIX="media/videos/"
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".h264"

FILE=$PREFIX$CAMTYPE$DATE$SUFFIX

sudo raspivid -o $FILE -t $LENGTH
