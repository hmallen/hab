#!/bin/bash

PREFIX="~/icarus_one/media/videos/UP-"
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".mp4"

FILE=$PREFIX$DATE$SUFFIX

fswebcam --no-banner -q -i /dev/video1 -o $FILE
