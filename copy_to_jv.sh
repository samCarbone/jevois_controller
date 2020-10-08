#!/usr/bin/env bash

echo "streamoff" > /dev/ttyACM0 && \
sleep 1

# First read any accumulated junk in the serial buffers:
while true; do
sudo bash -c "read -s -t 0.05 -s < \"/dev/ttyACM0\""
if [ $? -ne 0 ]; then break; fi
done

sleep 1

jevois-usbsd start && \
sleep 1 && \
cp ./pbuild/Main /media/samuel/JEVOIS/scripts && \
cp ./launch-platform.sh /media/samuel/JEVOIS/scripts && \
sleep 1 && \
jevois-usbsd stop && \
echo "Successful copy"


