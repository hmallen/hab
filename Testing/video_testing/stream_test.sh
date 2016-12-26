#!/bin/bash

WIDTH=$1
HEIGHT=$2
FPS=$3

#raspivid -o -t 0 -hf -w $WIDTH -h $HEIGHT -fps $FPS | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8160}' :demux=h264
raspivid -o - -t 0 -fps $FPS -w $WIDTH -h $HEIGHT | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8554/}' :demux=h264 :h264-fps=$FPS
