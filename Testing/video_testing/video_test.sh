#!/bin/sh

sudo streamer -c /dev/video0 -s 640x480 -f jpeg -t 60 -r 12 -j 75 -w 0 -o tmp/tmp.avi &&
sudo avconv -i tmp/tmp.avi -preset ultrafast -crf 27 output.mp4 &&
sudo rm tmp/tmp.avi
