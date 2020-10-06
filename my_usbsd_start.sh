#!/usr/bin/env bash

echo "streamoff" > /dev/ttyACM0 && \
sleep 1 && \
jevois-usbsd start


