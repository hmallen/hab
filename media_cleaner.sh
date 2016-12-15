#!/usr/bin/env bash

PREFIX="media_"
DATE=$(date +"%m%d%Y-%H%M%S")
ARCHIVE=$PREFIX$DATE

mv media archive/$ARCHIVE
cp -R archive/template/media/ media
