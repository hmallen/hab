#!/bin/bash

VIDEOSOURCE=$0
((LENGTHSEC=$1))
((FPS=12))
((LENGTH=FPS*LENGTHSEC))

PREFIX="~/icarus_one/media/videos/DOWN-"
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".mp4"

FILE=$PREFIX$DATE$SUFFIX

sudo streamer -c /dev/video0 -s 640x480 -f jpeg -t $LENGTH -r $FPS -j 75 -w 0 -o tmp/tmp.avi &&
sudo avconv -i tmp/tmp.avi -preset ultrafast -crf 27 $FILE &&
sudo rm tmp/tmp.avi
