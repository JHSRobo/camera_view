#!/bin/bash

# Script to be run at EVERY boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

grep -q "Setup" /etc/rc.local || ( echo "Please run setup!" && exit )

# Start up camera streamer
cd /home/pi/rpicamera/streamer
bash raspivid.sh &
bash ping.sh &
bash version.sh &
#bash `find /home/pi -iname raspivid.sh || find /home/jhsrobo -iname raspivid.sh | head -1` &
#bash `find /home/pi -iname ping.sh || find /home/jhsrobo -iname ping.sh | head -1` &
#bash `find /home/pi -iname version.sh || find /home/jhsrobo -iname version.sh | head -1` &
