#!/usr/bin/env bash

jevois-usbsd start && \
cp ./pbuild/Main /media/samuel/JEVOIS/scripts && \
cp ./launch-platform.sh /media/samuel/JEVOIS/scripts && \
sleep 5 && \
jevois-usbsd stop && \
echo "Successful copy"


