#!/usr/bin/env bash

PREFIX="media_"
DATE=$(date +"%m%d%Y-%H%M%S")
ARCHIVE=$PREFIX$DATE

mv media archive/$ARCHIVE
mv logs archive/$ARCHIVE/logs
cp -R archive/template/media/ media
cp -R archive/template/logs/ logs

