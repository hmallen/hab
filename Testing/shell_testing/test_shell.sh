#!/bin/sh

PREFIX="UP-"
DATE=$(date +"%Y-%m-%d_%H%M%S")
SUFFIX=".jpg"
FULLNAME=$PREFIX$DATE$SUFFIX

echo $FULLNAME
