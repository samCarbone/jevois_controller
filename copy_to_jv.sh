#!/usr/bin/env bash

echo "streamoff" > /dev/ttyACM0 && \
sleep 1 && \
jevois-usbsd start && \
sleep 1 && \
cp ./pbuild/Main /media/samuel/JEVOIS/scripts && \
cp ./launch-platform.sh /media/samuel/JEVOIS/scripts && \
sleep 1 && \
jevois-usbsd stop && \
echo "Successful copy"


