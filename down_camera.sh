#!/bin/sh

PREFIX="DOWN-"
SUFFIX=".jpg"
DATE=$(date +"%Y-%m-%d_%H%M%S")

FILENAME=$PREFIX$DATE$SUFFIX

fswebcam --no-banner -q -i /dev/video1 -o ~/icarus_one/media/photos/$FILENAME.jpg
