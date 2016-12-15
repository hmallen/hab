#!/usr/bin/env bash

LENGTHSEC=$2
FPS=15
((LENGTH=FPS*LENGTHSEC))

if [ "$1" = "0" ]
then
CAMTYPE="UP-"
else
CAMTYPE="DOWN-"
fi

PREFIX="media/videos/"
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".avi"

FILE=$PREFIX$CAMTYPE$DATE$SUFFIX

sudo nohup streamer -c /dev/video$1 -s 640x480 -f jpeg -t $LENGTH -r $FPS -j 75 -w 0 -o $FILE > logs/webcam_video.out 2> logs/webcam_video.err < /dev/null &
#sudo nohup avconv -i tmp/tmp.avi -preset ultrafast -crf 27 $FILE > webcam_video.out 2> webcam_video.err < /dev/null &
#sudo rm tmp/tmp.avi
