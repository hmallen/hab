#!/bin/sh

PREFIX="~/icarus_one/media/photos/UP-"
SUFFIX=".jpg"
DATE=$(date +"%Y-%m-%d_%H%M%S")

FILE=$PREFIX$DATE$SUFFIX

fswebcam --no-banner -q -i /dev/video1 -o $FILE
