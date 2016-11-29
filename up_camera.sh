#!/bin/sh

PREFIX="~/icarus_one/media/photos/UP-"
DATE=$(date +"%Y-%m-%d_%H%M%S")
SUFFIX=".jpg"

FILE=$PREFIX$DATE$SUFFIX

fswebcam --no-banner -q -i /dev/video1 -o $FILE
