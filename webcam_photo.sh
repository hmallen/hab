#!/usr/bin/env bash

if [ "$1" = "0" ]
then
CAMTYPE="UP-"
else
CAMTYPE="DOWN-"
fi

PREFIX="media/photos/"
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".jpg"

FILE=$PREFIX$CAMTYPE$DATE$SUFFIX

fswebcam --no-banner -d /dev/video$1 -r 1920x1080 -S 30 $FILE
