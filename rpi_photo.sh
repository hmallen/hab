#!/usr/bin/env bash

CAMTYPE="RPI-"

PREFIX="media/photos/"
DATE=$(date +"%m%d%Y-%H%M%S")
SUFFIX=".jpg"

FILE=$PREFIX$CAMTYPE$DATE$SUFFIX

raspistill -o $FILE
