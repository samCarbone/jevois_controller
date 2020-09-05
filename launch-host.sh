#!/bin/sh

save_state=$(stty -g) \
&& stty -F /dev/ttyACM0 57600 -echo -echoe -echoctl -echoke raw \
&& echo -e "\r\nStarted [REMOVE]\r\n" > /dev/ttyACM0 \
&& echo -e "********START ERR LOG*********" > ~/Documents/host_logs/err_log.txt \
&& ./hbuild/Main -f /dev/ttyACM0 \
; echo -e "\r\nFinished [REMOVE]\r\n" > /dev/ttyACM0 \
; stty "$save_state" 

# >> ./hbuild/logs/err_log.txt 2>&1 \