#!/usr/bin/sh 

acm_ports="$(ls /dev/ | grep ttyACM)"
if [ -z "${acm_ports}" ]; then 
    >&2 echo "No serial ports found."
fi 

printf "/dev/%s\n" $acm_ports 

