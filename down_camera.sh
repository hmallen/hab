#!/bin/sh

PREFIX="~/icarus_one/media/photos/DOWN-"
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".jpg"

FILE=$PREFIX$DATE$SUFFIX

fswebcam --no-banner -q -i /dev/video1 -o $FILE
