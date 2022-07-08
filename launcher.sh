#!/bin/sh
# launcher.sh
# navigate to home directory, then to this directory, the execute the python script, then back home

cd /
cd /home/pi/workspace/NGTbot
sudo pm2 start shock.py --interpreter python3 -- -dw

