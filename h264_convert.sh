#!/usr/bin/env bash

FILEINPUT=$1
FILEOUTPUT=$2

sudo nohup avconv -i $FILEINPUT -preset ultrafast -crf 27 $FILEOUTPUT &
