#!/bin/bash

LENGTHSEC=$2
FPS=15
((LENGTH=FPS*LENGTHSEC))

if [ "$1" = "0" ]
then
CAMTYPE="UP-"
else
CAMTYPE="DOWN-"
fi

PREFIXDIR="media/videos/"
PREFIX=$CAMTYPE$PREFIXDIR
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".mp4"

FILE=$PREFIX$CAMTYPE$DATE$SUFFIX

sudo streamer -c /dev/video$1 -s 640x480 -f jpeg -t $LENGTH -r $FPS -j 75 -w 0 -o tmp/tmp.avi &&
sudo avconv -i tmp/tmp.avi -preset ultrafast -crf 27 $FILE &&
sudo rm tmp/tmp.avi
