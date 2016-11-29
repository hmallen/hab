#!/bin/bash

if [ "$1" = "0" ]
then
CAMTYPE="UP-"
else
CAMTYPE="DOWN-"
fi

PREFIXDIR="media/photos/"
PREFIX=$PREFIXDIR$CAMTYPE
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".jpg"

FILE=$PREFIX$DATE$SUFFIX

sudo fswebcam --no-banner -d /dev/video$1 -r 1280x720 $FILE
