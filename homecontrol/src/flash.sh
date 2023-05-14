#!/bin/bash
#
# Flash script for HomeControl AtMega48 using avrdude
#

# Flash using avrdude on Raspberry PI
#sudo avrdude -c linuxgpio -p atmega48 -v -U flash:w:control.hex:i

# Flash using Pololu ISP USB programmer (use pavr2gui / pavr2cmd to change VOut settings)
sudo avrdude -c avrisp2 -P /dev/ttyACM0 -p atmega48 -v -U flash:w:control.hex:i

