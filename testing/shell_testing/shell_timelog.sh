#!/usr/bin/env bash

DATE=$(date +"%m%d%Y-%H%M%S")
FILE="/home/pi/icarus_one/logs/shell_timelog.txt"

echo $DATE >> $FILE
