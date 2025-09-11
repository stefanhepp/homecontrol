#!/bin/sh

# Expecting pixtendlibv2 installed in pi user's .venv
. /home/pi/.venv/bin/activate

python `dirname $0`/pixcontrol.py
