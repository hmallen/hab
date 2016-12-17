#!/usr/bin/env bash

PREFIX="media_"
DATE=$(date +"%m%d%Y-%H%M%S")
ARCHIVE=$PREFIX$DATE

mv media archive/$ARCHIVE
mv logs archive/$ARCHIVE/logs
mv mode_file.txt archive/$ARCHIVE
cp -R archive/template/media/ media
cp -R archive/template/logs/ logs
cp archive/template/mode_file.txt mode_file.txt
