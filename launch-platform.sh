#!/bin/sh

# save_state=$(stty -g) && stty -F /dev/ttyS0 115200 -echo -echoe -echoctl -echoke raw && stty -F /dev/ttyS0 -a > /dev/ttyS0 && python3 /jevois/scripts/my_echo.py &> /dev/ttyS0 < /dev/ttyS0 ; stty "$save_state"
# && python3 /jevois/scripts/echo_3.py ; \
# && stty -F /dev/ttyS0 -a > /dev/ttyS0 \

save_state=$(stty -g) \
&& sleep 2 \
&& stty -F /dev/ttyS0 57600 -echo -echoe -echoctl -echoke raw \
&& echo -e "\r\nStarted [REMOVE]\r\n" > /dev/ttyS0 \
&& echo -e "********START ERR LOG*********" > /jevois/scripts/logs/err_log.txt \
&& /jevois/scripts/Main -f /dev/ttyS0 >> /jevois/scripts/logs/err_log.txt 2>&1 \
; echo -e "\r\nFinished [REMOVE]\r\n" > /dev/ttyS0 \
; stty "$save_state" 
