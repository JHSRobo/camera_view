#!/bin/bash

# Script to be run ONCE at the first boot

# Checks sudo perms needed for installing libraries, editing files and changing passwords
if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

# Pings google. if google breaks, we don't have access to start a new camera, sorry. Needs the internet and
# this was the best way I could think of. If you have a better way to for it
# TODO
if [[ "$(echo -e "GET http://google.com HTTP/1.0\n\n" | nc google.com 80 > /dev/null 2>&1)" == 0 ]]; then
    echo "Please connect to the internet to install libraries"
    exit
fi

# Makes sure the PI is up to date. This could cause things to break in the future
apt update -y

# Checks if ncat and nmap are installed and then installs them if they are not
# Needed for the raspivid function to stream the camera
dpkg -l | grep ncat || apt install ncat -y
dpkg -l | grep nmap || apt install nmap -y


# Safety checks
# Dont wanna go through that again
# Assumes that if the device name is pi then it must be a pi0
if [[ -d "/home/pi" ]]
then
  # change the password for the pi account
  # cuz safety lol but plaintext
  echo -e "JHSRobo\nJHSRobo" | passwd pi

  # enable cameras
  raspi-config nonint do_camera 0

  # give more memory
  echo -e "$(sed '/gpu_mem/d' /boot/config.txt)" > /boot/config.txt
  echo "gpu_mem=256" >> /boot/config.txt

  # turn off the red light. if you leave it on, it reflects off the glass
  echo "disable_camera_led=1" >> /boot/config.txt

  # reboot
  echo "#Setup" >> /etc/rc.local
  reboot now
else
  # Setup for the main pi instead of the camera pis
  # We use the crontab instead of RC local to work better with ROS and ubuntu mate instead of raspbian
  # doesn't rename the pi and doesn't restart it which should allow it to not break the whole thing
  # "Whoops lol" - Andrew

  # edit rc.local
  echo -en "@reboot bash "; echo -n "$(find "/home/jhsrobo" -iname "rpicamera" | head -1)"; echo -en "/streamer/startup.sh" > /var/spool/cron/crontabs/root

  # reboot
  echo "Setup complete, please reboot"
  echo "#Setup" >> /etc/rc.local
  exit

fi
