#!/usr/bin/sh 

if [ ! $1 ]; then 
    >&2 echo "Must specify device"
    exit 1
fi 

if [ ! -e "$1" ]; then 
    >&2 printf "The file %s cannot be read\n" $1 
    exit 2
fi 

if sh -c ": > $1" > /dev/null 2>/dev/null; then 
    stty -F "$1" 115200 -brkint igncr -imaxbel
    printf "%s configured with settings: \n" "$1"
    stty -F "$1" 
else 
    >&2 printf "The file %s cannot be accessed.\n" $1
    exit 3 
fi 


