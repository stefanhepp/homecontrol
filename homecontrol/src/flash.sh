#!/bin/bash
#
# Flash script for HomeControl AtMega48 using avrdude on RaspberryPi
#
sudo avrdude -c linuxgpio -p atmega48 -v -U flash:w:control.hex:i
