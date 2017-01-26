#!/usr/bin/env bash

DATE=$(date +"%m%d%Y-%H%M%S")
LOGPREFIX="Reboot @ "
LOGSTRING=$LOGPREFIX$DATE
FILE="/home/pi/icarus_one/logs/shell_rebootlog.txt"

echo $LOGSTRING >> $FILE
